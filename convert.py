#!/usr/bin/env python3
"""convert.py — 将 ROS2 URDF（支持完整全身模型）转换为自包含、可直接用 MuJoCo viewer 打开的成品 MJCF。

把原本用于 ROS/RViz 的 URDF（例如 openflex_integrated_robot.urdf：移动底盘 + 升降 + 双臂 + 头部）
编译成 MuJoCo 模型，并注入：
  * 地板 + 灯光（场景自带，viewer 直接打开即可看）
  * 夹爪联动 ``<equality>``（URDF 的 ``<mimic>`` 被 MuJoCo 编译时丢弃，这里补回）
  * 位置 actuator：双臂 14 关节 + 2 主手指 + 头部 2 + 升降 1 + 转向 4，nu=23
    （finger_joint2 由 equality 约束驱动，轮子为连续关节不加 actuator 仅靠阻尼稳定）

网格统一收敛到 ``mujoco_meshes/``，成品 XML 用相对路径引用，因此可直接拷贝给
任何装了 MuJoCo 的环境用最普通的 viewer 显示，无需中间文件。

``package://<pkg>/`` 前缀会自动映射到本地 ``packages/<pkg>/`` 目录，因此多 package 的
完整模型也能直接转换。

用法:
    python convert.py            # 生成 openflex_mujoco.xml
    python convert.py --check    # 仅生成并校验，不进入交互

仅当源 URDF 或网格需要更新时重跑本脚本即可。
"""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
SOURCE_URDF = ROOT / "openflex_integrated_robot.urdf"
OUTPUT_XML = ROOT / "openflex_mujoco.xml"
# 全面自碰撞版本（使用 ROS 原始 <collision> 作为碰撞几何）
OUTPUT_SELFCOL_XML = ROOT / "openflex_mujoco_selfcol.xml"
MUJOCO_MESH_DIR = ROOT / "mujoco_meshes"
PACKAGES_DIR = ROOT / "packages"

# 把 package://<pkg>/ 映射到本地的 packages/ 下任意层级的 <pkg>/ 目录。
# 支持子模块嵌套布局（例如 swerve_description 实际位于 base_model_interface_layer 仓库内）。
PACKAGE_PREFIXES: dict[str, Path] = {}

def _discover_packages(root: Path) -> None:
    if not root.is_dir():
        return
    for _sub in sorted(root.iterdir()):
        if not _sub.is_dir() or _sub.name == ".git":
            continue
        # 目录名本身即一个 ROS package（如 swerve_description / openarmx_description ...）
        if _sub.name.endswith("_description") and (
            (_sub / "urdf").is_dir() or (_sub / "package.xml").is_file()
        ):
            PACKAGE_PREFIXES.setdefault(f"package://{_sub.name}/", _sub)
        # 继续向下一级递归，兼容子模块嵌套（如 packages/openflex_chassis/swerve_description）
        _discover_packages(_sub)

_discover_packages(PACKAGES_DIR)

# 整个机器人绕 Z 轴旋转，使机器人正对默认视角
ROBOT_YAW = "-1.57079632679"

# --------------------------------------------------------------------------- #
# 执行器增益（沿用双臂版本的臂/手指参数，并为升降/头部/转向补充合理值）
# --------------------------------------------------------------------------- #
ARM_SERVO_KP = 180
FINGER_SERVO_KP = 80
HEAD_SERVO_KP = 40
LIFT_SERVO_KP = 10000          # 升降要扛约 253N 重力负载，高刚度稳住（略降以减少灵敏/猛冲）
STEER_SERVO_KP = 20

# position actuator 的微分增益（即用户提到的 kd）：提供速度阻尼，消除振荡、增强可控性
ARM_SERVO_KV = 20
FINGER_SERVO_KV = 8
HEAD_SERVO_KV = 4
LIFT_SERVO_KV = 2000          # ≈ 2×临界阻尼(2*sqrt(kp*Meff)≈1000)，过阻尼彻底消除振荡、快速稳定
STEER_SERVO_KV = 4

# 关节物理参数：damping / armature / frictionloss
ARM_JOINT_DAMPING = 18.0
ARM_JOINT_ARMATURE = 0.03
ARM_JOINT_FRICTION = 0.15
FINGER_JOINT_DAMPING = 2.0
FINGER_JOINT_ARMATURE = 0.002
FINGER_JOINT_FRICTION = 0.02
HEAD_JOINT_DAMPING = 2.0
HEAD_JOINT_ARMATURE = 0.01
HEAD_JOINT_FRICTION = 0.05
LIFT_JOINT_DAMPING = 60.0
LIFT_JOINT_ARMATURE = 0.1
LIFT_JOINT_FRICTION = 2.0
STEER_JOINT_DAMPING = 3.0
STEER_JOINT_ARMATURE = 0.005
STEER_JOINT_FRICTION = 0.1
WHEEL_JOINT_DAMPING = 1.0
WHEEL_JOINT_ARMATURE = 0.02
WHEEL_JOINT_FRICTION = 0.0

# MuJoCo 接触/约束求解器参数（夹爪联动用）
EQUALITY_SOLIMP = "0.95 0.99 0.001"
EQUALITY_SOLREF = "0.005 1"


def _qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用四元数 [w,x,y,z] 旋转一组向量 (N,3)。"""
    w, x, y, z = q
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def compute_min_z(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """计算当前位姿下所有网格几何的世界最低点 z（用于把机器人抬到地板面）。"""
    mujoco.mj_forward(model, data)
    min_z = np.inf
    for i in range(model.ngeom):
        did = int(model.geom_dataid[i])
        if did < 0:
            continue
        adr = int(model.mesh_vertadr[did])
        num = int(model.mesh_vertnum[did])
        verts = np.asarray(model.mesh_vert[adr:adr + num])
        gp = np.asarray(model.geom_pos[i])
        gq = np.asarray(model.geom_quat[i])
        bid = int(model.geom_bodyid[i])
        bp = np.asarray(data.xpos[bid])
        bq = np.asarray(data.xquat[bid])
        loc = _qrot(gq, verts) + gp
        world = _qrot(bq, loc) + bp
        min_z = min(min_z, float(world[:, 2].min()))
    return min_z


def classify_joint(name: str) -> str | None:
    """把关节名归类为物理/执行器处理类别。"""
    if name.endswith("finger_joint2"):
        return "finger_mimic"          # 由 equality 约束驱动，不加 actuator
    if "finger_joint1" in name:
        return "finger"
    if name.startswith("openarmx_left_joint") or name.startswith("openarmx_right_joint"):
        return "arm"
    if name.startswith("openarmx_head_"):
        return "head"
    if name == "lift_joint":
        return "lift"
    if "steering" in name:
        return "steer"
    if "wheel" in name:
        return "wheel"                 # 连续关节，仅靠阻尼稳定
    return None


JOINT_PHYSICS: dict[str, tuple[float, float, float]] = {
    "arm":    (ARM_JOINT_DAMPING, ARM_JOINT_ARMATURE, ARM_JOINT_FRICTION),
    "finger": (FINGER_JOINT_DAMPING, FINGER_JOINT_ARMATURE, FINGER_JOINT_FRICTION),
    "head":   (HEAD_JOINT_DAMPING, HEAD_JOINT_ARMATURE, HEAD_JOINT_FRICTION),
    "lift":   (LIFT_JOINT_DAMPING, LIFT_JOINT_ARMATURE, LIFT_JOINT_FRICTION),
    "steer":  (STEER_JOINT_DAMPING, STEER_JOINT_ARMATURE, STEER_JOINT_FRICTION),
    "wheel":  (WHEEL_JOINT_DAMPING, WHEEL_JOINT_ARMATURE, WHEEL_JOINT_FRICTION),
}

ACTUATOR_KP: dict[str, float] = {
    "arm": ARM_SERVO_KP,
    "finger": FINGER_SERVO_KP,
    "head": HEAD_SERVO_KP,
    "lift": LIFT_SERVO_KP,
    "steer": STEER_SERVO_KP,
}

ACTUATOR_KV: dict[str, float] = {
    "arm": ARM_SERVO_KV,
    "finger": FINGER_SERVO_KV,
    "head": HEAD_SERVO_KV,
    "lift": LIFT_SERVO_KV,
    "steer": STEER_SERVO_KV,
}


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
    # 关闭固定关节融合：让每个 link 都成为独立 body（名字即 link 名），
    # 这样自碰撞版能按 link 名定位 body 并注入 ROS <collision> 几何。
    compiler_tag.set("fusestatic", "false")


def convert_package_uri(filename: str) -> str:
    for prefix, base in PACKAGE_PREFIXES.items():
        if filename.startswith(prefix):
            return str(base / filename.removeprefix(prefix))
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
    """把 'packages/x/meshes/arm/v10/visual/link0.dae' 摊平成唯一文件名。"""
    base = "_".join(rel.with_suffix("").parts)
    suffix = scale_suffix(scale)
    if part_index is not None:
        return f"{base}{suffix}_part{part_index}.obj"
    return f"{base}{suffix}.obj"  # 统一输出 obj


def convert_visual_mesh(filename: str, scale: list[float], urdf_color: str | None = None
                        ) -> list[tuple[str, str | None]]:
    """返回 [(相对 ROOT 的 mesh 路径, 颜色), ...]，并把网格写到 mujoco_meshes/。

    所有网格都经 trimesh 转成 .obj（dae 提取自身 diffuse 颜色；stl 等用 URDF 内联材质颜色），
    这样 ASCII / 二进制 STL 都能被正确处理，并统一出口格式。
    """
    source_path = Path(convert_package_uri(filename))
    if not source_path.exists():
        raise FileNotFoundError(source_path)
    rel = source_path.relative_to(ROOT)
    is_dae = source_path.suffix.lower() == ".dae"
    colors = read_dae_diffuse_colors(source_path) if is_dae else []

    mesh_or_scene = trimesh.load(source_path)
    meshes = mesh_or_scene.dump(concatenate=False) if isinstance(mesh_or_scene, trimesh.Scene) else [mesh_or_scene]
    outputs = []
    for i, part in enumerate(meshes):
        flat = flat_mesh_name(rel, scale, part_index=i if len(meshes) > 1 else None)
        out_path = MUJOCO_MESH_DIR / flat
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if not out_path.exists():
            print(f"🔄 转换 {source_path.suffix} -> obj: {source_path.name} [{i}] -> {flat}")
            bake_negative_scale(part.copy(), scale).export(out_path)
        color = (colors[i] if i < len(colors) else (colors[-1] if colors else None)) if is_dae else urdf_color
        outputs.append((f"mujoco_meshes/{flat}", color))
    return outputs


def set_visual_material(visual: ET.Element, color: str | None, name: str) -> None:
    old = visual.find("material")
    if old is not None:
        visual.remove(old)
    if not color:
        return
    material = ET.Element("material", {"name": name})
    ET.SubElement(material, "color", {"rgba": color})
    visual.append(material)


def _urdf_visual_color(visual: ET.Element) -> str | None:
    """提取 visual 内联材质颜色（如 URDF 中 <material><color rgba='...'/></material>）。"""
    mat = visual.find("material")
    if mat is None:
        return None
    color = mat.find("color")
    if color is None:
        return None
    return color.get("rgba")


def generate_visual_urdf(source_urdf: Path, output_urdf: Path) -> Path:
    """去掉 collision、剥离 ros2_control、把每个 visual mesh 换成 MuJoCo 友好的引用（相对路径）。"""
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    ensure_mujoco_compiler(root)

    # MuJoCo 的 URDF 解析器不认识 <ros2_control>，必须先剥离
    for rc in list(root.findall("ros2_control")):
        root.remove(rc)

    for link in root.findall("link"):
        # 运动连杆必须有质量/惯量，否则 MuJoCo 报 mjMINVAL 错误；
        # 给缺失 <inertial> 的 link 补一个合理的默认值（视觉连杆常漏写）
        if link.find("inertial") is None:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            ET.SubElement(inertial, "mass", {"value": "0.5"})
            ET.SubElement(inertial, "inertia",
                          {"ixx": "0.002", "iyy": "0.002", "izz": "0.002",
                           "ixy": "0", "ixz": "0", "iyz": "0"})

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
            urdf_color = _urdf_visual_color(visual)
            parts = convert_visual_mesh(filename, scale, urdf_color)
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
<geom name="floor" type="plane" pos="0 0 0" size="2.4 2.4 0.02" material="floor_mat" contype="1" conaffinity="3"/>"""


# --------------------------------------------------------------------------- #
# 编译 + 注入
# --------------------------------------------------------------------------- #
def apply_joint_dynamics(root: ET.Element) -> None:
    for joint in root.findall(".//joint"):
        name = joint.get("name", "")
        kind = classify_joint(name)
        if kind is None or kind == "finger_mimic":
            continue
        damping, armature, friction = JOINT_PHYSICS[kind]
        joint.set("damping", str(damping))
        joint.set("armature", str(armature))
        joint.set("frictionloss", str(friction))


def _add_actuators_and_equality(root: ET.Element, model: mujoco.MjModel) -> tuple[int, int]:
    """夹爪联动 equality + position actuator（两个 builder 共用）。"""
    equality = root.find("equality")
    if equality is None:
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

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    added = 0
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if not name:
            continue
        kind = classify_joint(name)
        if kind is None or kind not in ACTUATOR_KP:
            continue
        lo, hi = model.jnt_range[jid]
        kp = ACTUATOR_KP[kind]
        kv = ACTUATOR_KV[kind]
        ET.SubElement(actuator, "position", {
            "name": f"{name}_pos",
            "joint": name,
            "kp": str(kp),
            "kv": str(kv),
            "ctrllimited": "true",
            "ctrlrange": f"{lo} {hi}",
        })
        added += 1
    return added_eq, added


def _add_contact_excludes(root: ET.Element) -> int:
    """相邻关节的父子 body 互不碰撞，避免关节处视觉/碰撞几何重叠导致的抖炸。

    非相邻的（如左臂↔右臂、臂↔机身/底盘、臂↔升降柱）仍会碰撞。
    """
    contact_el = root.find("contact")
    if contact_el is None:
        contact_el = ET.SubElement(root, "contact")
    # 标准库 ElementTree 没有 getparent()，用父节点映射
    parent_map = {child: parent for parent in root.iter() for child in parent}
    added = 0
    for body in root.iter("body"):
        parent = parent_map.get(body)
        if parent is not None and parent.tag == "body":
            pn = parent.get("name")
            bn = body.get("name")
            if pn and bn:
                ET.SubElement(contact_el, "exclude", {"body1": pn, "body2": bn})
                added += 1
    return added


def build_base_mjcf() -> tuple[ET.ElementTree, mujoco.MjModel, float]:
    """编译『视觉版』URDF -> MJCF，并完成地板/灯光/yaw root 装配。

    返回 (tree, model, base_lift)。该 MJCF 中机器人几何全部来自 <visual>（仅渲染），
    碰撞分组由各自 builder 后续设置；model 仍保留关节信息供 actuator 生成使用。
    """
    # 1) URDF -> 中间 URDF（相对路径网格，剥离 ros2_control 与 collision）
    tmp_urdf = ROOT / "_tmp_convert.urdf"
    generate_visual_urdf(SOURCE_URDF, tmp_urdf)

    # 2) 编译成模型，再序列化为 MJCF（含 body 层级、joint、asset mesh）
    model = mujoco.MjModel.from_xml_path(str(tmp_urdf))
    # 计算机器人几何最低点，用于把底盘抬到地板面（避免一半陷入地下）
    _tmp_data = mujoco.MjData(model)
    min_z = compute_min_z(model, _tmp_data)
    base_lift = round(-min_z, 4)   # 上抬量：使最低点落到 z=0
    print(f"📐 几何最低点 z={min_z:.4f} → 机器人整体上抬 {base_lift:.4f}")
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
    yaw = ET.SubElement(worldbody, "body",
                        {"name": "openflex_robot_yaw_root",
                         "pos": f"0 0 {base_lift}",
                         "euler": f"0 0 {ROBOT_YAW}"})
    for c in robot_children:
        yaw.append(c)
    lights_root = ET.fromstring(f"<root>{FLOOR_LIGHTS}</root>")
    for child in lights_root:
        worldbody.append(child)

    return tree, model, base_lift


def build_runtime_xml() -> Path:
    """版本 A：仅与地板/外物碰撞，连杆之间关闭自碰撞（当前默认版本）。"""
    tree, model, _ = build_base_mjcf()
    root = tree.getroot()

    # 机器人视觉网格 geom: contype=2(bit1, 仅属 robot 组) conaffinity=1(bit0, 只与世界组碰)
    # 地板 geom: contype=1 conaffinity=3(世界+robot 组)
    for geom in root.findall(".//geom"):
        if geom.get("type") == "mesh":
            geom.set("contype", "2")
            geom.set("conaffinity", "1")

    apply_joint_dynamics(root)
    eq, act = _add_actuators_and_equality(root, model)
    tree.write(OUTPUT_XML, encoding="utf-8", xml_declaration=True)
    print(f"✅ [版本A:仅地面碰撞] 夹爪联动约束: {eq} | position actuator: {act}")
    return OUTPUT_XML


def build_selfcol_xml() -> Path:
    """版本 B：双臂/升降/机身/底盘之间全面自碰撞。

    优化（消除冗余）：直接让『视觉网格 geom 本身』参与碰撞——MuJoCo 的 mesh geom
    本来就同时负责渲染与碰撞，contype/conaffinity 只决定碰不碰。因此无需再生成一套
    与视觉网格重叠的碰撞网格（之前那版既重复又需透明隐藏）。

    好处：
    - 零冗余：每个 link 一个 geom 同时渲染+碰撞，删掉重复 obj 与透明 hack；
    - 底盘大平台（base_link 视觉网格）自动获得碰撞体积，手臂推到机身上会被挡住；
    - 相邻关节的父子 body 加 <exclude>，避免关节处几何重叠抖炸；
    - 非相邻的（左臂↔右臂、臂↔机身/底盘、臂↔升降柱）正常碰撞。

    代价：碰撞使用精细视觉网格的三角形（精确但比简化碰撞网格稍慢），viewer 交互足够。
    """
    tree, model, _ = build_base_mjcf()
    root = tree.getroot()

    # 视觉网格 geom 由『只碰地板(2/1)』改为『与一切碰撞(3/3)』，开启自碰撞。
    for geom in root.findall(".//geom"):
        if geom.get("type") == "mesh":
            geom.set("contype", "3")
            geom.set("conaffinity", "3")
    floor = root.find(".//geom[@name='floor']")
    if floor is not None:
        floor.set("contype", "1")
        floor.set("conaffinity", "3")

    excl = _add_contact_excludes(root)
    apply_joint_dynamics(root)
    eq, act = _add_actuators_and_equality(root, model)
    tree.write(OUTPUT_SELFCOL_XML, encoding="utf-8", xml_declaration=True)
    print(f"✅ [版本B:全面自碰撞] 视觉网格直接碰撞 | 相邻关节 exclude: {excl} | 夹爪联动: {eq} | actuator: {act}")
    return OUTPUT_SELFCOL_XML


def main() -> None:
    parser = argparse.ArgumentParser(description="OpenFlex ROS2 URDF -> MuJoCo 成品 XML")
    parser.add_argument("--check", action="store_true", help="只生成并校验，不进入交互")
    parser.add_argument("--with-selfcol", action="store_true",
                        help="同时生成全面自碰撞版本 openflex_mujoco_selfcol.xml（需要 packages/ 子模块就位）")
    args = parser.parse_args()

    out = build_runtime_xml()
    # 全面自碰撞版本：默认也生成（依赖 packages/ 子模块），失败仅警告
    try:
        build_selfcol_xml()
    except Exception as e:  # packages 缺失等情况
        print(f"⚠️ 跳过自碰撞版本生成（如需请先 git submodule update --init --recursive）: {e}")

    model = mujoco.MjModel.from_xml_path(str(out))
    print(f"✅ 成品 XML 可加载: nq={model.nq} nv={model.nv} nu={model.nu}")
    print(f"   产物: {out.name} | 网格目录: {MUJOCO_MESH_DIR.name}/")
    if args.check:
        return
    data = mujoco.MjData(model)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
