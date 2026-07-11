#!/usr/bin/env python3
"""从上游 xacro 源文件编译 openflex_integrated_robot.urdf

不依赖完整 ROS2 环境，仅需 pip install xacro。
通过注入假的 ament_index_python 模块，让 xacro 的 $(find ...) 解析到本地 packages/ 子模块。
"""

import os
import sys
import shutil
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
PACKAGES = HERE / "packages"

# xacro 主入口文件
XACRO_INPUT = PACKAGES / "openarmx_integrated_description" / "urdf" / "openarmx_integrated_robot.urdf.xacro"

# 输出 URDF 路径（与 convert.py 输入一致）
OUTPUT = HERE / "openflex_integrated_robot.urdf"

# ROS 包名 → 本地路径映射
# $(find swerve_description) → packages/openflex_chassis/swerve_description
PKG_MAP = {
    "swerve_description":                 PACKAGES / "openflex_chassis" / "swerve_description",
    "lift_slide_description":             PACKAGES / "lift_slide_description",
    "openarmx_description":               PACKAGES / "openarmx_description",
    "openarmx_head_description":          PACKAGES / "openarmx_head_description",
    "openarmx_integrated_description":    PACKAGES / "openarmx_integrated_description",
}

# ---- 创建 share 目录符号链接 + 注入 fake ament_index_python ----


def _inject_fake_ament() -> None:
    """在 sys.modules 中注入假的 ament_index_python，使 xacro 找到本地包。

    xacro 的 $(find pkg) 依赖 ament_index_python.packages.get_package_share_directory()。
    这里构造最小实现，直接根据 PKG_MAP 返回本地路径。
    """
    # 避免重复注入
    if "ament_index_python" in sys.modules:
        return

    share_map: dict[str, str] = {}
    for pkg_name, pkg_path in PKG_MAP.items():
        if pkg_path.exists():
            share_map[pkg_name] = str(pkg_path.resolve())

    ament_index = types.ModuleType("ament_index_python")
    ament_index.__path__ = []

    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.__package__ = "ament_index_python"

    def get_package_share_directory(package_name: str) -> str:
        if package_name in share_map:
            return share_map[package_name]
        raise FileNotFoundError(
            f"package '{package_name}' not found. "
            f"Did you run: git submodule update --init --recursive --remote ?"
        )

    ament_packages.get_package_share_directory = get_package_share_directory

    sys.modules["ament_index_python"] = ament_index
    sys.modules["ament_index_python.packages"] = ament_packages


# ---- 主流程 ----


def check_prerequisites() -> None:
    """检查输入文件是否存在。"""
    if not XACRO_INPUT.exists():
        print(f"错误: 找不到 xacro 输入文件: {XACRO_INPUT}", file=sys.stderr)
        print("请先拉取子模块: git submodule update --init --recursive --remote", file=sys.stderr)
        sys.exit(1)

    try:
        import xacro  # noqa: F401
    except ImportError:
        print("错误: 未安装 xacro，请执行: pip install xacro", file=sys.stderr)
        sys.exit(1)


def build_urdf() -> None:
    """编译 xacro → URDF。"""
    check_prerequisites()
    _inject_fake_ament()

    import xacro

    print(f"编译 {XACRO_INPUT.name} ...")
    try:
        doc = xacro.process_file(str(XACRO_INPUT))
    except Exception as e:
        print(f"xacro 编译失败: {e}", file=sys.stderr)
        sys.exit(1)

    urdf_text = doc.toprettyxml(indent="  ")
    OUTPUT.write_text(urdf_text, encoding="utf-8")

    size_kb = OUTPUT.stat().st_size / 1024
    print(f"✓ 已生成: {OUTPUT} ({size_kb:.1f} KB)")


if __name__ == "__main__":
    build_urdf()
