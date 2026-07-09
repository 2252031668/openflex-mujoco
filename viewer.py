#!/usr/bin/env python3
"""viewer.py — 用最普通的 MuJoCo viewer 打开已转换好的成品 XML。

无需任何中间文件：直接读取 convert.py 生成的 openflex_mujoco.xml，
右侧面板拖滑块即可控制关节（14 臂 + 2 主手指），鼠标拖拽旋转/缩放。

用法:
    python viewer.py
    python viewer.py --check    # 只校验可加载，不打开窗口
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
OUTPUT_XML = ROOT / "openflex_mujoco.xml"


def seed_actuator_controls(model, data) -> None:
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        qpos_id = int(model.jnt_qposadr[joint_id])
        data.ctrl[actuator_id] = data.qpos[qpos_id]


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex MuJoCo 原生 viewer")
    parser.add_argument("--check", action="store_true", help="只校验可加载，不打开窗口")
    args = parser.parse_args()

    if not OUTPUT_XML.exists():
        raise SystemExit(f"找不到 {OUTPUT_XML.name}，请先运行 python convert.py")

    model = mujoco.MjModel.from_xml_path(str(OUTPUT_XML))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0.0
    seed_actuator_controls(model, data)

    print(f"✅ 已加载: nq={model.nq} nv={model.nv} nu={model.nu}")
    if args.check:
        return

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
