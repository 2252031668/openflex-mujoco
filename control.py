#!/usr/bin/env python3
"""OpenFlex MuJoCo control viewer using the ROS2 visual mesh model."""

from __future__ import annotations

import argparse
import math
import signal
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from view_ros2_visual_meshes import OUTPUT_URDF, SOURCE_URDF, generate_visual_urdf
from scene_utils import FLOOR_SCENE

# Global flag for graceful shutdown via Ctrl+C
_running = True


BASE_DIR = Path(__file__).resolve().parent
RUNTIME_XML = BASE_DIR / "openflex_control_runtime.xml"
SCENE_XML = BASE_DIR / "openflex_control_scene.xml"

LEFT_FINGER_BODIES = ["openarmx_left_right_finger", "openarmx_left_left_finger"]
RIGHT_FINGER_BODIES = ["openarmx_right_right_finger", "openarmx_right_left_finger"]

LEFT_FINGER_JOINTS = ["openarmx_left_finger_joint1", "openarmx_left_finger_joint2"]
RIGHT_FINGER_JOINTS = ["openarmx_right_finger_joint1", "openarmx_right_finger_joint2"]

ARM_JOINT_PREFIXES = ("openarmx_left_joint", "openarmx_right_joint")
GRIPPER_SMOOTH = 0.18
ARM_SERVO_KP = 180
ARM_JOINT_DAMPING = 18.0
ARM_JOINT_ARMATURE = 0.03
ARM_JOINT_FRICTION = 0.15
FINGER_JOINT_DAMPING = 2.0
FINGER_JOINT_ARMATURE = 0.002
FINGER_JOINT_FRICTION = 0.02
ROBOT_YAW_LEFT_90 = "-1.57079632679"


def name_to_id(model, obj_type, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"找不到对象: {name}")
    return obj_id


def joint_qpos(model, joint_id: int) -> int:
    return int(model.jnt_qposadr[joint_id])


def clamp_joint(model, joint_id: int, value: float) -> float:
    if model.jnt_limited[joint_id]:
        low, high = model.jnt_range[joint_id]
        return float(np.clip(value, low, high))
    return float(value)


def joint_open_close(model, joint_id: int, margin_ratio: float = 0.08) -> tuple[float, float]:
    if model.jnt_limited[joint_id]:
        low, high = [float(v) for v in model.jnt_range[joint_id]]
    else:
        low, high = 0.0, 0.044
    margin = (high - low) * margin_ratio
    return low + margin, high - margin


def get_body_center(model, data, body_names: list[str]) -> np.ndarray:
    positions = []
    for body_name in body_names:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id >= 0:
            positions.append(data.xpos[body_id].copy())
    return np.mean(positions, axis=0) if positions else np.zeros(3)


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

    for joint in root.findall(".//joint"):
        joint_name = joint.get("name", "")
        if joint_name.startswith(ARM_JOINT_PREFIXES):
            joint.set("damping", str(ARM_JOINT_DAMPING))
            joint.set("armature", str(ARM_JOINT_ARMATURE))
            joint.set("frictionloss", str(ARM_JOINT_FRICTION))
        elif "finger" in joint_name:
            joint.set("damping", str(FINGER_JOINT_DAMPING))
            joint.set("armature", str(FINGER_JOINT_ARMATURE))
            joint.set("frictionloss", str(FINGER_JOINT_FRICTION))

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    else:
        for child in list(actuator):
            actuator.remove(child)

    added = 0
    for joint_id in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not joint_name or "finger" in joint_name:
            continue
        if not joint_name.startswith(ARM_JOINT_PREFIXES):
            continue
        low, high = model.jnt_range[joint_id]
        ET.SubElement(
            actuator,
            "position",
            {
                "name": f"{joint_name}_pos",
                "joint": joint_name,
                "kp": str(ARM_SERVO_KP),
                "ctrllimited": "true",
                "ctrlrange": f"{low} {high}",
            },
        )
        added += 1

    tree.write(RUNTIME_XML, encoding="utf-8", xml_declaration=True)
    print(f"✅ 运行时模型: {RUNTIME_XML}")
    print(f"✅ position actuator 数量: {added}")
    return RUNTIME_XML


def write_scene(runtime_xml: Path) -> Path:
    """Write a scene that includes the runtime XML + mocap viz bodies."""
    scene_tree = ET.fromstring(FLOOR_SCENE)

    worldbody = scene_tree.find("worldbody")
    if worldbody is not None:
        # Gripper center viz spheres
        lv = ET.SubElement(worldbody, "body", {"name": "left_gripper_center_viz", "pos": "0 0 0", "mocap": "true"})
        ET.SubElement(lv, "geom", {"name": "left_gripper_center_viz", "type": "sphere",
                                   "size": "0.015", "rgba": "0 1 1 0.8",
                                   "contype": "0", "conaffinity": "0"})
        rv = ET.SubElement(worldbody, "body", {"name": "right_gripper_center_viz", "pos": "0 0 0", "mocap": "true"})
        ET.SubElement(rv, "geom", {"name": "right_gripper_center_viz", "type": "sphere",
                                   "size": "0.015", "rgba": "1 0.6 0 0.8",
                                   "contype": "0", "conaffinity": "0"})

    ET.SubElement(scene_tree, "include", {"file": str(runtime_xml)})

    SCENE_XML.write_text(ET.tostring(scene_tree, encoding="utf-8").decode(), encoding="utf-8")
    return SCENE_XML


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex ROS2 visual MuJoCo joint/gripper control.")
    parser.add_argument("--check", action="store_true", help="Only build and load the model")
    args = parser.parse_args()

    runtime_xml = generate_runtime_xml()
    scene_xml = write_scene(runtime_xml)
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)

    left_jids = [name_to_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in LEFT_FINGER_JOINTS]
    right_jids = [name_to_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in RIGHT_FINGER_JOINTS]
    left_qpos = [joint_qpos(model, jid) for jid in left_jids]
    right_qpos = [joint_qpos(model, jid) for jid in right_jids]

    left_open_close = [joint_open_close(model, jid) for jid in left_jids]
    right_open_close = [joint_open_close(model, jid) for jid in right_jids]

    left_viz_body = name_to_id(model, mujoco.mjtObj.mjOBJ_BODY, "left_gripper_center_viz")
    right_viz_body = name_to_id(model, mujoco.mjtObj.mjOBJ_BODY, "right_gripper_center_viz")
    left_viz_mocap = model.body_mocapid[left_viz_body]
    right_viz_mocap = model.body_mocapid[right_viz_body]

    state = {
        "left_open": True,
        "right_open": True,
    }

    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        data.ctrl[actuator_id] = data.qpos[joint_qpos(model, joint_id)]

    model.opt.gravity[:] = 0.0
    mujoco.mj_forward(model, data)

    print("✅ OpenFlex 控制脚本已加载")
    print("快捷键: O/C 双夹爪开合, Z/X 左夹爪, N/M 右夹爪")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} ngeom={model.ngeom}")

    if args.check:
        return

    def key_callback(keycode: int) -> None:
        global _running
        try:
            key = chr(keycode).upper()
        except ValueError:
            return
        if key == "O":
            state["left_open"] = True
            state["right_open"] = True
            print("👉 双夹爪张开")
        elif key == "C":
            state["left_open"] = False
            state["right_open"] = False
            print("👉 双夹爪闭合")
        elif key == "Z":
            state["left_open"] = True
            print("👉 左夹爪张开")
        elif key == "X":
            state["left_open"] = False
            print("👉 左夹爪闭合")
        elif key == "N":
            state["right_open"] = True
            print("👉 右夹爪张开")
        elif key == "M":
            state["right_open"] = False
            print("👉 右夹爪闭合")
        elif key == "Q":
            _running = False
            print("👉 退出")

    signal.signal(signal.SIGINT, lambda sig, frame: setattr(
        __import__("sys").modules[__name__], "_running", False))

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [0.0, -0.02, 0.58]
        viewer.cam.distance = 2.25
        viewer.cam.elevation = -16
        viewer.cam.azimuth = 180

        while _running and viewer.is_running():
            # Apply gripper targets
            left_targets = [pair[1] if state["left_open"] else pair[0] for pair in left_open_close]
            right_targets = [pair[1] if state["right_open"] else pair[0] for pair in right_open_close]
            for qpos_id, joint_id, target in zip(left_qpos, left_jids, left_targets):
                data.qpos[qpos_id] += GRIPPER_SMOOTH * (target - data.qpos[qpos_id])
                data.qpos[qpos_id] = clamp_joint(model, joint_id, data.qpos[qpos_id])
            for qpos_id, joint_id, target in zip(right_qpos, right_jids, right_targets):
                data.qpos[qpos_id] += GRIPPER_SMOOTH * (target - data.qpos[qpos_id])
                data.qpos[qpos_id] = clamp_joint(model, joint_id, data.qpos[qpos_id])

            try:
                mujoco.mj_step(model, data)
                data.mocap_pos[left_viz_mocap] = get_body_center(model, data, LEFT_FINGER_BODIES)
                data.mocap_pos[right_viz_mocap] = get_body_center(model, data, RIGHT_FINGER_BODIES)
                viewer.sync()
            except Exception:
                break
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
