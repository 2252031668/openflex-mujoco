#!/usr/bin/env python3
"""viewer.py — 用 MuJoCo launch_passive 打开已转换好的成品 XML。

直接读取成品 XML，右侧面板拖滑块控制关节，鼠标拖拽旋转/缩放。
初始相机视角由文件顶部 CAM_* 配置块控制，打开即自由视角。

用法:
    python viewer.py                      # 默认：带手 + 全面自碰撞版本
    python viewer.py --plain              # 版本A：仅地面碰撞，无手
    python viewer.py --self-collision     # 版本B：全身自碰撞，原夹爪，无手
    python viewer.py --check              # 只校验可加载，不打开窗口
    python viewer.py --no-rt              # 关闭实时步长（循环全速运行）
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
OUTPUT_XML = ROOT / "openflex_mujoco.xml"
OUTPUT_SELFCOL_XML = ROOT / "openflex_mujoco_selfcol.xml"
OUTPUT_HAND_XML = ROOT / "openflex_mujoco_hand.xml"


# ===================== 相机初始视角（可调参数） =====================
# 想改默认观察角度，直接调下面这几个值即可（与你给的写法一一对应）：
CAM_TYPE      = mujoco.mjtCamera.mjCAMERA_FREE   # 自由相机：打开即鼠标旋转/缩放
CAM_LOOKAT    = None        # 焦点 [x, y, z]；None = 自动取「水平中心 + 胸部高度」
CAM_DISTANCE  = 2.0         # 相机离焦点的距离（米）
CAM_AZIMUTH   = 90.0        # 方位角(度)：0=+X 看向 -X；90=相机在 +Y 侧（机器人正前方）
CAM_ELEVATION = 0.0         # 俯仰角(度)：0=水平；负值=向下俯视
# ===================================================================


def seed_actuator_controls(model, data) -> None:
    """把每个执行器的控制量设成当前关节位置，使机器人初始保持静止姿态。"""
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        qpos_id = int(model.jnt_qposadr[joint_id])
        data.ctrl[actuator_id] = data.qpos[qpos_id]


def init_front_camera(model, data, cam) -> None:
    """把自由相机放到机器人正前方、与胸部齐平；参数全部由 CAM_* 配置控制。

    复用主循环的 data 做 mj_forward，不再额外分配 MjData。
    正面方向由头部前置 RGBD 相机 camera_link（位于最 -Y 侧）确认；胸高取 chest_link 世界高度。
    """
    if CAM_LOOKAT is None:
        mujoco.mj_forward(model, data)
        ys = data.xpos[:, 1]
        cy = float((ys.min() + ys.max()) / 2)          # 机器人水平中心 Y
        try:
            cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chest_link")
            chest_z = float(data.xpos[cid, 2])
        except Exception:
            zs = data.xpos[:, 2]
            chest_z = float((zs.min() + zs.max()) / 2)
        cam.lookat[:] = [0.0, cy, chest_z * 0.95]      # 自动焦点
    else:
        cam.lookat[:] = list(CAM_LOOKAT)               # 手动焦点

    cam.type = CAM_TYPE
    cam.distance = CAM_DISTANCE
    cam.azimuth = CAM_AZIMUTH
    cam.elevation = CAM_ELEVATION


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex MuJoCo 原生 viewer")
    parser.add_argument("--check", action="store_true", help="只校验可加载，不打开窗口")
    parser.add_argument("--plain", action="store_true",
                        help="加载版本A openflex_mujoco.xml（仅地面碰撞，无手）")
    parser.add_argument("--self-collision", action="store_true",
                        help="加载版本B openflex_mujoco_selfcol.xml（原夹爪 + 全身自碰撞，无手）")
    parser.add_argument("--no-rt", action="store_true",
                        help="关闭实时步长（默认按 model 时间步 sleep，物理以真实速度推进）")
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

    # launch_passive：窗口在独立线程、物理循环留在主线程，支持鼠标自由旋转/缩放。
    # 相机视角在「启动后、循环前」用代码初始化（参数见文件顶部 CAM_* 配置块）。
    viewer = mujoco.viewer.launch_passive(model, data)
    init_front_camera(model, data, viewer.cam)

    # 实时循环：按 model 时间步 sleep，使物理以真实速度推进，避免 CPU 满载。
    realtime = not args.no_rt
    while viewer.is_running():
        step_start = time.perf_counter()
        mujoco.mj_step(model, data)
        viewer.sync()
        if realtime:
            elapsed = time.perf_counter() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)


if __name__ == "__main__":
    main()
