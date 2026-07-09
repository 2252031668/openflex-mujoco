#!/usr/bin/env python3
"""Left-arm position IK demo using the ROS2 visual OpenFlex MuJoCo model."""

from __future__ import annotations

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from view_ros2_visual_meshes import OUTPUT_URDF, SOURCE_URDF, generate_visual_urdf
from scene_utils import FLOOR_SCENE


BASE_DIR = Path(__file__).resolve().parent
RUNTIME_XML = BASE_DIR / "openflex_ik_runtime.xml"
SCENE_XML = BASE_DIR / "openflex_ik_scene.xml"

FINGER_BODIES = ["openarmx_left_right_finger", "openarmx_left_left_finger"]
JACOBIAN_BODY = "openarmx_left_link7"
ARM_JOINTS = [f"openarmx_left_joint{i}" for i in range(1, 8)]
GRIPPER_JOINTS = ["openarmx_left_finger_joint1", "openarmx_left_finger_joint2"]

GRIPPER_OPEN = 0.040
GRIPPER_CLOSE = 0.002
GRIPPER_SMOOTH = 0.15

DAMPING = 0.20
K_POS = 1.2
K_NULL = 0.001
MAX_DQ = 0.02
Q_REST = np.array([0.0, -0.35, 0.15, 0.85, 0.0, 0.45, 0.0], dtype=np.float64)

TARGET_MIN = np.array([-0.45, -0.55, 0.02], dtype=np.float64)
TARGET_MAX = np.array([0.80, 0.55, 1.05], dtype=np.float64)
ROBOT_YAW_LEFT_90 = "-1.57079632679"


def name_to_id(model, obj_type, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"找不到对象: {name}")
    return obj_id


def clamp_qpos(model, data, qpos_ids: list[int], joint_ids: list[int]) -> None:
    for qpos_id, joint_id in zip(qpos_ids, joint_ids):
        if model.jnt_limited[joint_id]:
            low, high = model.jnt_range[joint_id]
            data.qpos[qpos_id] = np.clip(data.qpos[qpos_id], low, high)


def get_body_pos(model, data, body_name: str) -> np.ndarray:
    body_id = name_to_id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[body_id].copy()


def get_gripper_center(model, data) -> np.ndarray:
    positions = []
    for body_name in FINGER_BODIES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id >= 0:
            positions.append(data.xpos[body_id].copy())
    if not positions:
        raise ValueError(f"找不到手指 body: {FINGER_BODIES}")
    return np.mean(positions, axis=0)


def clamp_target(pos: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(pos, TARGET_MIN), TARGET_MAX)


def solve_position_ik(
    model,
    data,
    body_id: int,
    target_pos: np.ndarray,
    current_pos: np.ndarray,
    arm_dofs: list[int],
    arm_qpos: list[int],
) -> tuple[np.ndarray, np.ndarray]:
    error = target_pos - current_pos
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, current_pos, body_id)

    jac = jacp[:, arm_dofs]
    damping_matrix = (DAMPING**2) * np.eye(3)
    jac_damped_pinv = jac.T @ np.linalg.solve(jac @ jac.T + damping_matrix, np.eye(3))
    dq_primary = jac_damped_pinv @ (K_POS * error)

    null_projector = np.eye(len(arm_dofs)) - jac_damped_pinv @ jac
    dq_secondary = K_NULL * (Q_REST - data.qpos[arm_qpos])
    dq = dq_primary + null_projector @ dq_secondary

    norm = np.linalg.norm(dq)
    if norm > MAX_DQ and norm > 1e-12:
        dq = dq / norm * MAX_DQ

    return dq, error


def generate_runtime_xml() -> Path:
    visual_urdf = generate_visual_urdf(SOURCE_URDF, OUTPUT_URDF)
    model = mujoco.MjModel.from_xml_path(str(visual_urdf))
    mujoco.mj_saveLastXML(str(RUNTIME_XML), model)

    tree = ET.parse(RUNTIME_XML)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is not None:
        robot_root = ET.Element("body", {"name": "openflex_robot_yaw_root", "euler": f"0 0 {ROBOT_YAW_LEFT_90}"})
        for child in list(worldbody):
            worldbody.remove(child)
            robot_root.append(child)
        worldbody.append(robot_root)
    tree.write(RUNTIME_XML, encoding="utf-8", xml_declaration=True)

    return RUNTIME_XML


def write_scene(runtime_xml: Path) -> Path:
    """Write a scene that includes the runtime XML + IK target + viz spheres."""
    scene_tree = ET.fromstring(FLOOR_SCENE)

    worldbody = scene_tree.find("worldbody")
    if worldbody is not None:
        # IK target (red ball) - mocap so user can Ctrl+right-click drag to perturb
        it = ET.SubElement(worldbody, "body", {"name": "ik_target", "pos": "0.25 0.18 0.45", "mocap": "true"})
        ET.SubElement(it, "geom", {"name": "ik_target", "type": "sphere",
                                   "size": "0.035", "rgba": "1 0 0 0.45",
                                   "contype": "0", "conaffinity": "0"})
        # Gripper center viz (cyan)
        gv = ET.SubElement(worldbody, "body", {"name": "gripper_center_viz", "pos": "0 0 0", "mocap": "true"})
        ET.SubElement(gv, "geom", {"name": "gripper_center_viz", "type": "sphere",
                                   "size": "0.015", "rgba": "0 1 1 0.8",
                                   "contype": "0", "conaffinity": "0"})

        # Target move handles (WASD + Q/E)
        tg = ET.SubElement(worldbody, "body", {"name": "target_move_handle", "pos": "0 0 0", "mocap": "true"})
        ET.SubElement(tg, "geom", {"name": "target_move_handle", "type": "box",
                                   "size": "0.02 0.02 0.02", "rgba": "0 0 0 0",
                                   "contype": "0", "conaffinity": "0"})

    ET.SubElement(scene_tree, "include", {"file": str(runtime_xml)})

    SCENE_XML.write_text(ET.tostring(scene_tree, encoding="utf-8").decode(), encoding="utf-8")
    return SCENE_XML


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex ROS2 visual MuJoCo left-arm IK.")
    parser.add_argument("--check", action="store_true", help="Only build and load the model")
    args = parser.parse_args()

    runtime_xml = generate_runtime_xml()
    scene_xml = write_scene(runtime_xml)
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)

    jac_body_id = name_to_id(model, mujoco.mjtObj.mjOBJ_BODY, JACOBIAN_BODY)
    target_body_id = name_to_id(model, mujoco.mjtObj.mjOBJ_BODY, "ik_target")
    target_mocap_id = model.body_mocapid[target_body_id]
    viz_body_id = name_to_id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_center_viz")
    viz_mocap_id = model.body_mocapid[viz_body_id]

    arm_joint_ids = [name_to_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINTS]
    arm_dofs = [int(model.jnt_dofadr[joint_id]) for joint_id in arm_joint_ids]
    arm_qpos = [int(model.jnt_qposadr[joint_id]) for joint_id in arm_joint_ids]

    gripper_joint_ids = []
    for joint_name in GRIPPER_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id >= 0:
            gripper_joint_ids.append(joint_id)
    gripper_qpos = [int(model.jnt_qposadr[joint_id]) for joint_id in gripper_joint_ids]

    state = {
        "pause": False,
        "debug": True,
        "gripper_target": GRIPPER_OPEN,
        "target_pos": np.array([0.25, 0.18, 0.45], dtype=np.float64),
    }

    def reset_robot() -> None:
        data.qpos[arm_qpos] = Q_REST.copy()
        clamp_qpos(model, data, arm_qpos, arm_joint_ids)
        for qpos_id in gripper_qpos:
            data.qpos[qpos_id] = GRIPPER_OPEN
        mujoco.mj_forward(model, data)
        target_pos = clamp_target(get_gripper_center(model, data).copy())
        data.mocap_pos[target_mocap_id] = target_pos
        state["target_pos"] = target_pos.copy()
        data.mocap_pos[viz_mocap_id] = target_pos
        mujoco.mj_forward(model, data)

    def key_callback(keycode: int) -> None:
        try:
            key = chr(keycode).upper()
        except ValueError:
            return

        step = 0.03
        if key == "W":
            state["target_pos"][1] -= step  # forward (Y- in rotated frame)
        elif key == "S":
            state["target_pos"][1] += step  # backward
        elif key == "A":
            state["target_pos"][0] -= step  # left
        elif key == "D":
            state["target_pos"][0] += step  # right
        elif key == "Q":
            state["target_pos"][2] -= step  # down
        elif key == "E":
            state["target_pos"][2] += step  # up
        elif key == "O":
            state["gripper_target"] = GRIPPER_OPEN
            print("👉 左夹爪张开")
        elif key == "C":
            state["gripper_target"] = GRIPPER_CLOSE
            print("👉 左夹爪闭合")
        elif key == "R":
            reset_robot()
            print("👉 复位")
        elif key == "P":
            state["pause"] = not state["pause"]
            print(f"👉 IK {'暂停' if state['pause'] else '继续'}")
        elif key == "I":
            state["debug"] = not state["debug"]
            print(f"👉 调试输出 {'开启' if state['debug'] else '关闭'}")
        else:
            return
        state["target_pos"] = clamp_target(state["target_pos"])
        data.mocap_pos[target_mocap_id] = state["target_pos"].copy()

    model.opt.gravity[:] = 0.0
    reset_robot()

    print("✅ OpenFlex IK 脚本已加载")
    print("操作: WASD 移动红球, Q/E 上下, O/C 夹爪开合, R 复位, P 暂停, I 调试, 鼠标拖拽旋转/缩放")
    print(f"nq={model.nq} nv={model.nv} nbody={model.nbody} ngeom={model.ngeom}")

    if args.check:
        return

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [0.0, -0.02, 0.58]
        viewer.cam.distance = 2.25
        viewer.cam.elevation = -16
        viewer.cam.azimuth = 180

        last_debug = 0.0
        while viewer.is_running():
            target_pos = clamp_target(state["target_pos"])
            data.mocap_pos[target_mocap_id] = target_pos

            data.qvel[:] = 0.0
            data.qacc[:] = 0.0
            mujoco.mj_forward(model, data)

            current_pos = get_gripper_center(model, data)
            if not state["pause"]:
                dq, error = solve_position_ik(model, data, jac_body_id, target_pos, current_pos, arm_dofs, arm_qpos)
                data.qpos[arm_qpos] += dq
                clamp_qpos(model, data, arm_qpos, arm_joint_ids)

            for qpos_id in gripper_qpos:
                data.qpos[qpos_id] += GRIPPER_SMOOTH * (state["gripper_target"] - data.qpos[qpos_id])

            mujoco.mj_forward(model, data)
            gripper_center = get_gripper_center(model, data)
            data.mocap_pos[viz_mocap_id] = gripper_center

            now = time.time()
            if state["debug"] and now - last_debug > 0.5:
                err = np.linalg.norm(target_pos - gripper_center)
                print(f"[DEBUG] target={target_pos.round(4)} cur={gripper_center.round(4)} err={err:.5f}")
                last_debug = now

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
