#!/usr/bin/env python3
"""convert.py — 将 ROS2 URDF 转换为自包含、可直接用 MuJoCo viewer 打开的成品 MJCF。

把原本用于 ROS/RViz 的 ``openflex_robot.urdf`` 编译成 MuJoCo 模型，并注入：
  * 地板 + 灯光（场景自带，viewer 直接打开即可看）
  * 夹爪联动 ``<equality>``（URDF 的 ``<mimic>`` 被 MuJoCo 编译时丢弃，这里补回）
  * 位置 actuator（14 个臂关节 + 2 个主手指 finger_joint1，nu=16；
    finger_joint2 由 equality 约束驱动，不单独加 actuator）

网格统一收敛到 ``mujoco_meshes/``，成品 XML 用相对路径引用，因此可直接拷贝给
任何装了 MuJoCo 的环境用最普通的 viewer 显示，无需中间文件。

用法:
    python convert.py            # 生成 openflex_mujoco.xml
    python convert.py --check    # 仅生成并校验，不进入交互

仅当 openflex_robot.urdf 或网格需要更新时重跑本脚本即可。
"""

from __future__ import annotations

import argparse
import copy
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import trimesh


ROOT = Path(__file__).resolve().parent
SOURCE_URDF = ROOT / "openflex_robot.urdf"
OUTPUT_XML = ROOT / "openflex_mujoco.xml"
MUJOCO_MESH_DIR = ROOT / "mujoco_meshes"
PACKAGE_PREFIX = "package://openarmx_description/"

# 整个机器人绕 Z 轴旋转，使双臂正对默认视角
ROBOT_YAW = "-1.57079632679"

# actuator 增益
ARM_SERVO_KP = 180
FINGER_SERVO_KP = 80
ARM_JOINT_DAMPING = 18.0
ARM_JOINT_ARMATURE = 0.03
ARM_JOINT_FRICTION = 0.15
FINGER_JOINT_DAMPING = 2.0
FINGER_JOINT_ARMATURE = 0.002
FINGER_JOINT_FRICTION = 0.02

# MuJoCo 接触/约束求解器参数（夹爪联动用）
EQUALITY_SOLIMP = "0.95 0.99 0.001"
EQUALITY_SOLREF = "0.005 1"

ARM_JOINT_PREFIXES = ("openarmx_left_joint", "openarmx_right_joint")


# --------------------------------------------------------------------------- #
# 网格转换：把 ROS2 的 dae/stl 网格拆成 MuJoCo 友好的 obj，并收敛到 mujoco_meshes/
# （逻辑来自原 view_ros2_visual_meshes.py，目录改为扁平的 mujoco_meshes/）
# --------------------------------------------------------------------------- #
def ensure_mujoco_compiler(root: ET.Element) -> None:
    mujoco_tag = root.find("mujoco")
    if mujoco_tag is None:
        mujoco_tag = ET.Element("mujoco")
        root.insert(0, mujoco_tag)
    compiler_tag = mujoco_tag.find("compiler")
    if compiler_tag is None:
        compiler_tag = ET.Element("compiler")
        mujoco_tag.append(compiler_tag)
    compiler_tag.set("strippath", "false")
    compiler_tag.set("discardvisual", "false")


def convert_package_uri(filename: str) -> str:
    if filename.startswith(PACKAGE_PREFIX):
        return str(ROOT / filename.removeprefix(PACKAGE_PREFIX))
    return filename


def parse_scale(scale_text: str) -> list[float]:
    if not scale_text:
        return [1.0, 1.0, 1.0]
    values = [float(v) for v in scale_text.split()]
    return values if len(values) == 3 else [1.0, 1.0, 1.0]


def scale_suffix(scale: list[float]) -> str:
    signs = ["n" if v < 0 else "p" for v in scale]
    return "_mirror_" + "".join(signs) if any(v < 0 for v in scale) else ""


def bake_negative_scale(mesh, scale: list[float]):
    if not any(v < 0 for v in scale):
        return mesh
    transform = [
        [-1.0 if scale[0] < 0 else 1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0 if scale[1] < 0 else 1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0 if scale[2] < 0 else 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    mesh.apply_transform(transform)
    return mesh


def read_dae_diffuse_colors(source_path: Path) -> list[str]:
    if source_path.suffix.lower() != ".dae":
        return []
    root = ET.parse(source_path).getroot()
    colors = []
    for element in root.iter():
        if element.tag.split("}")[-1] != "diffuse":
            continue
        for child in element:
            if child.tag.split("}")[-1] == "color" and child.text:
                colors.append(child.text.strip())
    return colors


def flat_mesh_name(rel: Path, scale: list[float], part_index: int | None = None) -> str:
    """把 'meshes/body/v10/visual/body_link0.dae' 摊平成唯一文件名
    'body_v10_visual_body_link0.obj'（dae 多 part 追加 _partN）。"""
    base = "_".join(rel.with_suffix("").parts)
    suffix = scale_suffix(scale)
    if part_index is not None:
        return f"{base}{suffix}_part{part_index}.obj"
    return f"{base}{suffix}{rel.suffix}"  # stl 保留原后缀


def convert_visual_mesh(filename: str, scale: list[float]) -> list[tuple[str, str | None]]:
    """返回 [(相对 ROOT 的 mesh 路径, 颜色), ...]，并把网格写到 mujoco_meshes/。"""
    source_path = Path(convert_package_uri(filename))
    if not source_path.exists():
        raise FileNotFoundError(source_path)
    try:
        rel = source_path.relative_to(ROOT / "meshes")
    except ValueError:
        rel = Path(source_path.name)

    if source_path.suffix.lower() == ".dae":
        colors = read_dae_diffuse_colors(source_path)
        mesh_or_scene = trimesh.load(source_path)
        meshes = mesh_or_scene.dump(concatenate=False) if isinstance(mesh_or_scene, trimesh.Scene) else [mesh_or_scene]
        outputs = []
        for i, part in enumerate(meshes):
            flat = flat_mesh_name(rel, scale, part_index=i)
            out_path = MUJOCO_MESH_DIR / flat
            out_path.parent.mkdir(parents=True, exist_ok=True)
            if not out_path.exists():
                print(f"🔄 转换 dae -> obj: {source_path.name} [{i}] -> {flat}")
                bake_negative_scale(part.copy(), scale).export(out_path)
            color = colors[i] if i < len(colors) else (colors[-1] if colors else None)
            outputs.append((f"mujoco_meshes/{flat}", color))
        return outputs

    # 非 dae（stl 等）：原样复制，扁平存放
    flat = flat_mesh_name(rel, scale)
    out_path = MUJOCO_MESH_DIR / flat
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not out_path.exists():
        print(f"📁 复制网格: {source_path.name} -> {flat}")
        shutil.copy2(source_path, out_path)
    return [(f"mujoco_meshes/{flat}", None)]


def set_visual_material(visual: ET.Element, color: str | None, name: str) -> None:
    old = visual.find("material")
    if old is not None:
        visual.remove(old)
    if not color:
        return
    material = ET.Element("material", {"name": name})
    ET.SubElement(material, "color", {"rgba": color})
    visual.append(material)


def generate_visual_urdf(source_urdf: Path, output_urdf: Path) -> Path:
    """去掉 collision、把每个 visual mesh 换成 MuJoCo 友好的 obj（相对路径）。"""
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    ensure_mujoco_compiler(root)

    for link in root.findall("link"):
        for collision in list(link.findall("collision")):
            link.remove(collision)
        for visual in list(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename", "")
            if not filename:
                continue
            scale = parse_scale(mesh.get("scale", ""))
            parts = convert_visual_mesh(filename, scale)
            link.remove(visual)
            for i, (mesh_path, color) in enumerate(parts):
                vp = copy.deepcopy(visual)
                if "name" in vp.attrib:
                    vp.set("name", f"{vp.attrib['name']}_part{i}")
                pm = vp.find("geometry/mesh")
                pm.set("filename", mesh_path)
                pm.set("scale", " ".join(str(abs(v)) for v in scale))
                set_visual_material(vp, color, f"{link.attrib.get('name', 'visual')}_mat_{i}")
                link.append(vp)

    output_urdf.write_text(ET.tostring(tree.getroot(), encoding="utf-8").decode(), encoding="utf-8")
    return output_urdf


# --------------------------------------------------------------------------- #
# 场景：地板 + 灯光（内联自原 scene_utils.FLOOR_SCENE）
# --------------------------------------------------------------------------- #
FLOOR_ASSET = """<asset>
  <texture name="sky" type="skybox" builtin="gradient"
           rgb1="0.08 0.10 0.14" rgb2="0.01 0.012 0.018" width="512" height="512"/>
  <texture name="floor_grid" type="2d" builtin="checker"
           rgb1="0.18 0.20 0.23" rgb2="0.26 0.29 0.33" width="1024" height="1024"
           mark="edge" markrgb="0.55 0.60 0.68"/>
  <material name="floor_mat" texture="floor_grid" texrepeat="8 8"
            reflectance="0.20" shininess="0.30" specular="0.20"/>
</asset>"""

FLOOR_VISUAL = """<visual>
  <headlight diffuse="0.55 0.55 0.55" ambient="0.24 0.24 0.26" specular="0.25 0.25 0.25"/>
  <map znear="0.01" zfar="8"/>
  <scale forcewidth="0.06" contactwidth="0.05"/>
  <rgba haze="0.03 0.035 0.045 1"/>
</visual>"""

FLOOR_LIGHTS = """<light name="key_light" pos="-1.4 -2.2 3.2" dir="0.35 0.55 -1" directional="true"
         diffuse="0.95 0.92 0.86" specular="0.35 0.35 0.35"/>
<light name="fill_light" pos="1.8 1.4 2.2" dir="-0.45 -0.35 -1" directional="true"
       diffuse="0.35 0.45 0.65" specular="0.12 0.12 0.16"/>
<light name="rim_light" pos="0 2.2 1.8" dir="0 -1 -0.45" directional="true"
       diffuse="0.55 0.65 0.85" specular="0.20 0.22 0.28"/>
<geom name="floor" type="plane" pos="0 0 -0.012" size="2.4 2.4 0.02" material="floor_mat"/>"""


# --------------------------------------------------------------------------- #
# 编译 + 注入
# --------------------------------------------------------------------------- #
def apply_joint_dynamics(root: ET.Element) -> None:
    for joint in root.findall(".//joint"):
        name = joint.get("name", "")
        if name.startswith(ARM_JOINT_PREFIXES):
            joint.set("damping", str(ARM_JOINT_DAMPING))
            joint.set("armature", str(ARM_JOINT_ARMATURE))
            joint.set("frictionloss", str(ARM_JOINT_FRICTION))
        elif "finger" in name:
            joint.set("damping", str(FINGER_JOINT_DAMPING))
            joint.set("armature", str(FINGER_JOINT_ARMATURE))
            joint.set("frictionloss", str(FINGER_JOINT_FRICTION))


def build_runtime_xml() -> Path:
    # 1) URDF -> 中间 URDF（相对路径网格）
    tmp_urdf = ROOT / "_tmp_convert.urdf"
    generate_visual_urdf(SOURCE_URDF, tmp_urdf)

    # 2) 编译成模型，再序列化为 MJCF（含 body 层级、joint、asset mesh）
    model = mujoco.MjModel.from_xml_path(str(tmp_urdf))
    tmp_xml = ROOT / "_tmp_convert.xml"
    mujoco.mj_saveLastXML(str(tmp_xml), model)
    tmp_urdf.unlink(missing_ok=True)

    tree = ET.parse(tmp_xml)
    root = tree.getroot()
    tmp_xml.unlink(missing_ok=True)

    # 3) mesh 路径相对化（saveLastXML 可能写出绝对路径，统一改成相对 ROOT 的 mujoco_meshes/ 引用）
    for mesh in root.findall(".//mesh"):
        f = mesh.get("file")
        if not f:
            continue
        mesh.set("file", f"mujoco_meshes/{Path(f).name}")

    # 4) 合并地板/灯光 asset + visual（并入已存在的对应节点）
    asset_el = root.find("asset")
    if asset_el is None:
        asset_el = ET.SubElement(root, "asset")
    for child in ET.fromstring(FLOOR_ASSET):
        asset_el.append(child)

    visual_el = root.find("visual")
    if visual_el is None:
        visual_el = ET.SubElement(root, "visual")
    for child in ET.fromstring(FLOOR_VISUAL):
        visual_el.append(child)

    # 5) worldbody：把机器人包进 yaw root，并加入灯光 + 地板
    worldbody = root.find("worldbody")
    robot_children = [
        c for c in list(worldbody)
        if not (c.tag == "light" or (c.tag == "geom" and c.get("name") == "floor"))
    ]
    for c in robot_children:
        worldbody.remove(c)
    yaw = ET.SubElement(worldbody, "body", {"name": "openflex_robot_yaw_root", "euler": f"0 0 {ROBOT_YAW}"})
    for c in robot_children:
        yaw.append(c)
    lights_root = ET.fromstring(f"<root>{FLOOR_LIGHTS}</root>")
    for child in lights_root:
        worldbody.append(child)

    apply_joint_dynamics(root)

    # 6) 夹爪联动 equality（joint2 跟随 joint1）
    equality = ET.SubElement(root, "equality")
    added_eq = 0
    for side in ("openarmx_left_", "openarmx_right_"):
        ET.SubElement(equality, "joint", {
            "joint1": f"{side}finger_joint1",
            "joint2": f"{side}finger_joint2",
            "solimp": EQUALITY_SOLIMP,
            "solref": EQUALITY_SOLREF,
        })
        added_eq += 1

    # 7) position actuator：14 臂关节 + 2 主手指（finger_joint1）
    actuator = ET.SubElement(root, "actuator")
    added = 0
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if not name:
            continue
        if name.endswith("finger_joint2"):
            continue
        if not (name.startswith(ARM_JOINT_PREFIXES) or "finger" in name):
            continue
        lo, hi = model.jnt_range[jid]
        kp = ARM_SERVO_KP if name.startswith(ARM_JOINT_PREFIXES) else FINGER_SERVO_KP
        ET.SubElement(actuator, "position", {
            "name": f"{name}_pos",
            "joint": name,
            "kp": str(kp),
            "ctrllimited": "true",
            "ctrlrange": f"{lo} {hi}",
        })
        added += 1

    tree.write(OUTPUT_XML, encoding="utf-8", xml_declaration=True)
    print(f"✅ 夹爪联动约束: {added_eq} | position actuator: {added}")
    return OUTPUT_XML


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex ROS2 URDF -> MuJoCo 成品 XML")
    parser.add_argument("--check", action="store_true", help="只生成并校验，不进入交互")
    args = parser.parse_args()

    out = build_runtime_xml()
    model = mujoco.MjModel.from_xml_path(str(out))
    print(f"✅ 成品 XML 可加载: nq={model.nq} nv={model.nv} nu={model.nu}")
    print(f"   产物: {out.name} | 网格目录: {MUJOCO_MESH_DIR.name}/")
    if args.check:
        return
    data = mujoco.MjData(model)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
