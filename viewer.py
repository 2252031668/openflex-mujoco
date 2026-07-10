#!/usr/bin/env python3
"""viewer.py — 用最普通的 MuJoCo viewer 打开已转换好的成品 XML。

无需任何中间文件：直接读取成品 XML，右侧面板拖滑块即可控制关节，鼠标拖拽旋转/缩放。

默认打开「带灵巧手 + 全面自碰撞」版本 (openflex_mujoco_hand.xml)：
    左手 20 关节 / 右手 20 关节 + 双臂 14

用法:
    python viewer.py                      # 默认：带手 + 全面自碰撞版本（左手/右手 + 全身互碰）
    python viewer.py --plain              # 版本A：仅与地板/外物碰撞（无手、无自碰撞）
    python viewer.py --self-collision     # 版本B：双臂/升降/机身/底盘全面自碰撞（原夹爪，无手）
    python viewer.py --check              # 只校验可加载，不打开窗口
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
OUTPUT_XML = ROOT / "openflex_mujoco.xml"
OUTPUT_SELFCOL_XML = ROOT / "openflex_mujoco_selfcol.xml"
OUTPUT_HAND_XML = ROOT / "openflex_mujoco_hand.xml"


def seed_actuator_controls(model, data) -> None:
    """把每个执行器的控制量设成当前关节位置，使机器人初始保持静止姿态。"""
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        qpos_id = int(model.jnt_qposadr[joint_id])
        data.ctrl[actuator_id] = data.qpos[qpos_id]


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

    # 最简单托管式 viewer：标准窗口、自带物理循环与渲染、鼠标可旋转/缩放。
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
