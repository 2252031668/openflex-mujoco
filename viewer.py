#!/usr/bin/env python3
"""viewer.py — 用最普通的 MuJoCo viewer 打开已转换好的成品 XML。

无需任何中间文件：直接读取成品 XML，右侧面板拖滑块即可控制关节，鼠标拖拽旋转/缩放。

默认打开「带灵巧手 + 全面自碰撞」版本 (openflex_mujoco_hand.xml)：
    左手 20 关节 / 右手 20 关节 + 双臂 14 + 头部 2 + 升降 1 + 转向 4，共 61 个执行器。

用法:
    python viewer.py                      # 默认：带手 + 全面自碰撞版本（左手/右手 + 全身互碰）
    python viewer.py --plain              # 版本A：仅与地板/外物碰撞（无手、无自碰撞）
    python viewer.py --self-collision     # 版本B：双臂/升降/机身/底盘全面自碰撞（原夹爪，无手）
    python viewer.py --check              # 只校验可加载，不打开窗口
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
OUTPUT_XML = ROOT / "openflex_mujoco.xml"
OUTPUT_SELFCOL_XML = ROOT / "openflex_mujoco_selfcol.xml"
OUTPUT_HAND_XML = ROOT / "openflex_mujoco_hand.xml"


# ===================== 相机初始视角（可调参数） =====================
# 想改默认观察角度，直接调下面这几个值即可（与自由相机 lookat/distance/azimuth/elevation 对应）：
CAM_LOOKAT    = None        # 焦点 [x, y, z]；None = 自动取「水平中心 + 胸部高度」
CAM_DISTANCE  = 4.0         # 相机离焦点的距离（米）
CAM_AZIMUTH   = -90.0       # 方位角(度)：0=+X 看向 -X；-90=相机在 -Y 侧（机器人正前方）
CAM_ELEVATION = 0.0         # 俯仰角(度)：0=水平；负值=向下俯视
CAM_FOVY      = 50.0        # 视野角度(度)
# ===================================================================


def seed_actuator_controls(model, data) -> None:
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        qpos_id = int(model.jnt_qposadr[joint_id])
        data.ctrl[actuator_id] = data.qpos[qpos_id]


def init_front_camera(model) -> None:
    """把模型里的 <camera name="front"> 设成「正面(-Y) + 胸高」并作为默认视角。

    你调的 CAM_LOOKAT/DISTANCE/AZIMUTH/ELEVATION 会被换算成固定相机的
    pos / xyaxes / fovy 写回模型，launch 启动即以此视角显示（窗口正常、可关闭）。
    """
    if CAM_LOOKAT is None:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        ys = data.xpos[:, 1]
        cy = float((ys.min() + ys.max()) / 2)          # 机器人水平中心 Y
        try:
            cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chest_link")
            chest_z = float(data.xpos[cid, 2])
        except Exception:
            zs = data.xpos[:, 2]
            chest_z = float((zs.min() + zs.max()) / 2)
        lookat = np.array([0.0, cy, chest_z * 0.95])
    else:
        lookat = np.array(CAM_LOOKAT, dtype=float)

    # 自由相机参数 -> 固定相机位姿
    a = np.deg2rad(CAM_AZIMUTH)
    e = np.deg2rad(CAM_ELEVATION)
    back = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    pos = lookat + CAM_DISTANCE * back
    zaxis = back / np.linalg.norm(back)                # 相机局部 +Z（指向背后）
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(world_up, zaxis)
    if np.linalg.norm(right) < 1e-6:
        right = np.cross(np.array([0.0, 1.0, 0.0]), zaxis)
    right = right / np.linalg.norm(right)
    up = np.cross(zaxis, right)
    xyaxes = np.concatenate([right, up])               # [right(3), up(3)]

    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "front")
    model.cam_pos[cid] = pos
    model.cam_xyaxes[cid] = xyaxes
    model.cam_fovy[cid] = CAM_FOVY
    model.vis.global_.cameraid = cid


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex MuJoCo 原生 viewer")
    parser.add_argument("--check", action="store_true", help="只校验可加载，不打开窗口")
    parser.add_argument("--plain", action="store_true",
                        help="加载版本A openflex_mujoco.xml（仅地面碰撞，无手）")
    parser.add_argument("--self-collision", action="store_true",
                        help="加载版本B openflex_mujoco_selfcol.xml（原夹爪 + 全身自碰撞，无手）")
    args = parser.parse_args()

    if args.plain:
        xml_path = OUTPUT_XML
        tag = "版本A:仅地面碰撞(无手)"
    elif args.self_collision:
        xml_path = OUTPUT_SELFCOL_XML
        tag = "版本B:全面自碰撞(原夹爪，无手)"
    else:
        xml_path = OUTPUT_HAND_XML
        tag = "带灵巧手 + 全面自碰撞"
    if not xml_path.exists():
        raise SystemExit(f"找不到 {xml_path.name}，请先运行对应构建脚本")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0.0
    seed_actuator_controls(model, data)

    print(f"已加载 [{tag}]: nq={model.nq} nv={model.nv} nu={model.nu}")
    if args.check:
        return

    # launch（托管式）：标准窗口、有关闭按钮、自带物理循环与渲染；鼠标可旋转/缩放。
    # 初始视角由 init_front_camera 把代码里的 CAM_* 参数写入模型 front 相机决定。
    init_front_camera(model)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
