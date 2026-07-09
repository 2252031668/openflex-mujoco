# OpenFlex MuJoCo 中文说明

本项目把原本用于 **ROS2 / RViz** 的 OpenFlex v10 双臂机械臂模型，转换成可直接用
**最普通的 MuJoCo viewer** 打开的成品 MJCF，并在此基础上提供双臂控制与夹爪联动。

核心思路：把「URDF → MuJoCo 编译 + 注入 actuator / 夹爪联动」这一步**一次性固化**成一个自包含成品 XML，
viewer 只负责加载显示，不再依赖任何运行时中间文件。

---

## 🎯 1. 项目目标

- 把 ROS2 / xacro / URDF 模型迁移到 MuJoCo 可加载模型
- 自动处理 `package://openarmx_description/...` 这类 ROS 路径
- 把 `.dae` visual mesh 转换 / 拆分 / 镜像烘焙后收敛到 `mujoco_meshes/`
- 把 URDF 的 `<mimic>`（被 MuJoCo 编译时丢弃）用 `<equality>` 约束补回，实现夹爪联动
- 生成一个自包含的成品 XML，可用任意 MuJoCo viewer 直接打开
- 提供原生 viewer（拖滑块控制关节）

---

## 🚀 2. 快速开始

本项目用 [uv](https://docs.astral.sh/uv/) 管理环境（也兼容普通 pip）：

```bash
uv sync
# 或： python3 -m pip install -r requirements.txt
```

环境自检：

```bash
python3 -c "import mujoco, mujoco.viewer, numpy, trimesh; print('MuJoCo environment OK')"
```

### 三步运行

```bash
# 1) 转换：ROS2 URDF -> 自包含成品 XML（生成 openflex_mujoco.xml + mujoco_meshes/）
python3 convert.py

# 2) 原生查看：用 MuJoCo 自带 viewer 打开成品 XML（右侧拖滑块控制关节）
python3 viewer.py
```

只校验生成 / 加载、不打开窗口：

```bash
python3 convert.py --check
python3 viewer.py  --check
```

---

## 📦 3. 依赖

- `mujoco`、`trimesh`、`pycollada`（见 `requirements.txt` / `pyproject.toml`）
- 若 `.dae` 转换失败，检查网格转换链：
  ```bash
  python3 -c "import trimesh, collada; print('mesh conversion deps OK')"
  ```

---

## 🗂️ 4. 项目结构

```bash
.
├── README_CN.md
├── README.md
├── convert.py                 # 转换程序：URDF -> 自包含成品 MJCF
├── viewer.py                  # 原生 MuJoCo viewer（加载成品 XML）
├── openflex_robot.urdf        # 源：ROS2 导出的原始 URDF
├── openflex_mujoco.xml        # 产物：自包含成品 MJCF（含 actuator/联动/地板/灯光）
├── meshes/                    # 源：ROS2 原始 mesh（.dae / .stl）
└── mujoco_meshes/             # 产物：转换后的 MuJoCo 友好网格（.obj / .stl，相对路径引用）
```

> `openflex_mujoco.xml` 与 `mujoco_meshes/` 都由 `convert.py` 生成；只有源 URDF / mesh 或参数
> 改动时才需要重跑 `convert.py`。`viewer.py` 不再产生任何中间文件。

---

## 📘 5. 核心文件说明

### `openflex_robot.urdf`

ROS2 导出的原始双臂 URDF，是整个工程的输入。其中夹爪用 `<mimic>` 描述联动，
但 MuJoCo 的 URDF 导入器会**静默丢弃**该标签，所以联动在转换时单独补回。

### `convert.py`

转换程序，只负责「编译 + 注入」，不涉及显示：

1. 去掉 collision、把每个 visual `.dae` 拆成 `.obj`（保留颜色），负缩放烘焙进网格
2. 编译成 MuJoCo 模型，序列化为 MJCF
3. 合并地板 + studio 风格灯光
4. 把机器人整体包进一个 `yaw root` body（绕 Z 轴旋转到正对默认视角）
5. 注入 `<equality>` 夹爪联动：每个 `finger_joint2` 跟随对应 `finger_joint1`
6. 注入 position actuator：14 个臂关节 + 2 个主手指（`finger_joint1`），**`nu=16`**
   （`finger_joint2` 由 equality 约束驱动，不加 actuator）
7. 网格路径统一收敛到 `mujoco_meshes/`，写成相对路径，成品 XML 可直接拷贝分发

产物 `openflex_mujoco.xml` 是**自包含**的，可被 `python -m mujoco.viewer openflex_mujoco.xml`
这类最普通的 MuJoCo viewer 直接打开。

### `viewer.py`

原生 viewer。直接 `mujoco.MjModel.from_xml_path(openflex_mujoco.xml)` +
`mujoco.viewer.launch(model, data)`，右侧面板拖滑块即可控制关节，鼠标拖拽旋转 / 缩放。

---

## 🤏 6. 夹爪联动设计

URDF 中每个夹爪有两个 prismatic 手指关节：

- `finger_joint1`：axis `0 -1 0`，range `0 0.044`
- `finger_joint2`：axis `0  1 0`，range `0 0.044`，并 `<mimic joint="finger_joint1"/>`

MuJoCo 导入 URDF 时丢弃 `<mimic>`，于是两个手指完全独立。`convert.py` 用
`<equality><joint joint1="finger_joint1" joint2="finger_joint2"/>` 约束
（默认 `q_joint2 = q_joint1`）。由于两关节 axis 符号相反、`q` 相等即产生对称的镜像开合，
且都在 `0 0.044` 合法范围内，不会出范围冲突。

控制界面每边只暴露一个滑块（`finger_joint1`），`finger_joint2` 由约束自动跟随。

---

## ⚠️ 7. 常见问题

### 1. MuJoCo 找不到 mesh 文件

- 必须在项目根目录运行脚本
- `meshes/`（源）或 `mujoco_meshes/`（产物）不完整时，重新运行 `python3 convert.py`
- URDF 中若仍有 `package://` 路径未转换，说明 convert 没跑成功

### 2. 夹爪不能闭合 / 不能拖动

确认用的是 `convert.py` 生成的 `openflex_mujoco.xml`，而不是旧 XML：

```bash
python3 convert.py --check
```

正常应看到 `nu=16`（14 臂 + 2 主手指）。若只有 `nu=14`，说明夹爪 actuator 没加进去，重跑 `convert.py`。

### 3. 模型在窗口里朝向不对

机器人整体朝向由 `convert.py` 里的 `ROBOT_YAW`（`-1.57079632679`，即绕 Z 轴 -90°）控制。
调整它即可改变机器人正面方向，而不只是改相机视角。

---

## 📁 8. 运行产物

- `openflex_mujoco.xml`：自包含成品 MJCF（含 actuator / 夹爪联动 / 地板 / 灯光）
- `mujoco_meshes/`：转换后的 MuJoCo 友好网格目录（相对路径被成品 XML 引用）

修改 URDF / mesh / 参数后，重新运行 `python3 convert.py` 刷新这两个产物即可。
