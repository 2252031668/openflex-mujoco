#!/usr/bin/env python3
"""build_hand_xml.py — 在「全身自碰撞版本」(openflex_mujoco_selfcol.xml) 基础上，
把 OpenFlex 原夹爪替换成 LDJY 灵巧手（左/右手），产出 openflex_mujoco_hand.xml。

做法（纯 XML 手术，不依赖 packages/ 子模块，直接读已提交的自碰撞成品）：
  1. 删掉每条臂末端的 OpenFlex 夹爪（openarmx_<side>_hand 子树 + finger_joint1/2
     执行器 + mimic 联动约束）。
  2. 读取 ldjy_hand/ldjy_<side>_hand.xml，把所有 body/joint/geom/mesh/actuator 名字
     加 openarmx_<side>_ 前缀（这样手被归入「机械臂」分组，能与机身/底盘正确自碰撞，
     且不会掉进「非机械臂刚体互碰」的排除里）。左手复用同一套 STL，仅靠 scale="-1 1 1" 镜像。
  3. 把手根 body（openarmx_<side>_palm）以 pos="0 0 0.1001"（即原夹爪挂载点）挂到
     openarmx_<side>_link7 末端。
  4. 手的视觉 geom 设为 contype=0/conaffinity=0（仅渲染），碰撞 geom（palm_collision、
     各指 *_collision、胶囊体）设为 3/3（参与自碰撞 + 碰地板）。
  5. 追加 40 个手指 position 执行器（左右手各 20）。
  6. 重算接触排除：相邻父子 body 互免（防手指/关节几何重叠抖炸）+ 非机械臂刚体互免
     （机身/底盘/升降/头部之间不互碰），完全复刻自碰撞版的逻辑并纳入手部。

运行: python build_hand_xml.py [--check]
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parent
SRC = ROOT / "openflex_mujoco_selfcol.xml"
OUT = ROOT / "openflex_mujoco_hand.xml"
HAND_DIR = ROOT / "ldjy_hand"

# 原夹爪挂载点（相对 link7），手根 palm 直接挂到这里
WRIST_POS = "0 0 0.1001"

# 机械臂分组前缀（决定哪些 body 算「机械臂」，从而参与自碰撞、不被非机械臂排除挡掉）
ARM_PREFIXES = ("openarmx_left_", "openarmx_right_")


def _parent_map(root: ET.Element) -> dict[ET.Element, ET.Element]:
    return {child: parent for parent in root.iter() for child in parent}


def load_hand(side: str) -> ET.Element:
    """读取单侧手 XML，重写网格路径并把所有名字加 openarmx_<side>_ 前缀。"""
    text = (HAND_DIR / f"ldjy_{side}_hand.xml").read_text(encoding="utf-8")
    # 网格路径：原 ../meshes/ -> 相对成品 XML（项目根）的 ldjy_hand/meshes/
    # （碰撞网格本在 ../meshes/collision/，一并拍平到 ldjy_hand/meshes/，先处理 collision 前缀）
    text = text.replace("../meshes/collision/", "ldjy_hand/meshes/")
    text = text.replace("../meshes/", "ldjy_hand/meshes/")
    # 前缀重命名（先整体替换，mesh 文件名里不含 left_/right_，故安全）
    text = text.replace(f"{side}_", f"openarmx_{side}_")
    return ET.fromstring(text)


def extract_hand_parts(hand_root: ET.Element):
    """从手 XML 抽出：mesh 定义、手根 body、position 执行器。丢弃 compiler/option/default/worldbody 壳。"""
    asset = hand_root.find("asset")
    mesh_defs = list(asset) if asset is not None else []
    act = hand_root.find("actuator")
    actuators = list(act) if act is not None else []
    wb = hand_root.find("worldbody")
    hand_body = list(wb)[0]  # 手根（openarmx_<side>_palm）
    return mesh_defs, hand_body, actuators


def set_hand_collision_groups(hand_body: ET.Element) -> None:
    """视觉 geom 仅渲染(0/0)，碰撞 geom(含 collision/capsule/body) 参与碰撞(3/3)。"""
    for geom in hand_body.iter("geom"):
        name = geom.get("name", "")
        if "visual" in name:
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
        else:
            geom.set("contype", "3")
            geom.set("conaffinity", "3")


def rebuild_contact(root: ET.Element) -> None:
    """重算 <contact>：相邻父子 body 互免 + 非机械臂刚体互免。"""
    contact_el = root.find("contact")
    if contact_el is None:
        contact_el = ET.SubElement(root, "contact")
    for ex in list(contact_el):
        contact_el.remove(ex)

    parent_map = _parent_map(root)
    # 1) 相邻父子 body 互免
    for body in root.iter("body"):
        parent = parent_map.get(body)
        if parent is not None and parent.tag == "body":
            pn, bn = parent.get("name"), body.get("name")
            if pn and bn:
                ET.SubElement(contact_el, "exclude", {"body1": pn, "body2": bn})
    # 2) 非机械臂刚体互免
    names = [b.get("name") for b in root.iter("body") if b.get("name")]
    non_arm = [n for n in names if n and not n.startswith(ARM_PREFIXES)]
    for i in range(len(non_arm)):
        for j in range(i + 1, len(non_arm)):
            ET.SubElement(contact_el, "exclude",
                          {"body1": non_arm[i], "body2": non_arm[j]})


def build() -> Path:
    if not SRC.exists():
        raise SystemExit(f"找不到 {SRC.name}，请先运行 python convert.py")

    tree = ET.parse(SRC)
    root = tree.getroot()
    parent_map = _parent_map(root)

    actuator_el = root.find("actuator")
    equality_el = root.find("equality")

    for side in ("left", "right"):
        # --- 删原夹爪 body ---
        gripper = root.find(f".//body[@name='openarmx_{side}_hand']")
        if gripper is not None:
            parent_map[gripper].remove(gripper)

        # --- 删原夹爪执行器（finger_joint1/2）---
        for a in list(actuator_el):
            if a.get("joint") in (f"openarmx_{side}_finger_joint1",
                                   f"openarmx_{side}_finger_joint2"):
                actuator_el.remove(a)

        # --- 删原 mimic 联动约束 ---
        if equality_el is not None:
            for e in list(equality_el):
                if e.get("joint1") == f"openarmx_{side}_finger_joint1":
                    equality_el.remove(e)

        # --- 插入手 ---
        hand_root = load_hand(side)
        mesh_defs, hand_body, actuators = extract_hand_parts(hand_root)
        hand_body.set("pos", WRIST_POS)
        set_hand_collision_groups(hand_body)

        # mesh 定义并入 asset（去掉 MuJoCo 不识别的 content_type，按扩展名自动识别 stl）
        asset_el = root.find("asset")
        for m in mesh_defs:
            m.attrib.pop("content_type", None)
            asset_el.append(m)

        # 手指执行器并入 actuator
        for a in actuators:
            actuator_el.append(a)

        # 手根挂到 link7 末端
        link7 = root.find(f".//body[@name='openarmx_{side}_link7']")
        link7.append(hand_body)

    rebuild_contact(root)
    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    return OUT


def validate(path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # 统计手相关 body / 碰撞 geom
    hand_bodies = sum(1 for i in range(model.nbody)
                      if (n := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i))
                      and (n.startswith("openarmx_left_") or n.startswith("openarmx_right_"))
                      and ("palm" in n or "finger" in n or "thumb" in n))
    print(f"✅ 带手版本可加载: nq={model.nq} nv={model.nv} nu={model.nu} | 手相关 body≈{hand_bodies}")
    print(f"   产物: {path.name}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只生成校验，不进交互")
    args = ap.parse_args()
    out = build()
    validate(out)
    if not args.check:
        data = mujoco.MjData(mujoco.MjModel.from_xml_path(str(out)))
        mujoco.viewer.launch(mujoco.MjModel.from_xml_path(str(out)), data)


if __name__ == "__main__":
    main()
