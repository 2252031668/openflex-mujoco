#!/usr/bin/env python3
"""viewer.py — 用最普通的 MuJoCo viewer 打开已转换好的成品 XML。

无需任何中间文件：直接读取 convert.py 生成的 openflex_mujoco.xml，
右侧面板拖滑块即可控制关节（14 臂 + 2 主手指），鼠标拖拽旋转/缩放。

用法:
    python viewer.py                      # 版本A：仅与地板/外物碰撞（默认）
    python viewer.py --self-collision     # 版本B：双臂/升降/机身/底盘之间全面自碰撞
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


def seed_actuator_controls(model, data) -> None:
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        qpos_id = int(model.jnt_qposadr[joint_id])
        data.ctrl[actuator_id] = data.qpos[qpos_id]


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex MuJoCo 原生 viewer")
    parser.add_argument("--check", action="store_true", help="只校验可加载，不打开窗口")
    parser.add_argument("--self-collision", action="store_true",
                        help="加载全面自碰撞版本 openflex_mujoco_selfcol.xml（双臂/升降/机身/底盘互碰）")
    args = parser.parse_args()

    xml_path = OUTPUT_SELFCOL_XML if args.self_collision else OUTPUT_XML
    if not xml_path.exists():
        raise SystemExit(f"找不到 {xml_path.name}，请先运行 python convert.py")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0.0
    seed_actuator_controls(model, data)

    tag = "版本B:全面自碰撞" if args.self_collision else "版本A:仅地面碰撞"
    print(f"✅ 已加载 [{tag}]: nq={model.nq} nv={model.nv} nu={model.nu}")
    if args.check:
        return

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
