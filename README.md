[📘 中文说明 (Chinese)](README_CN.md)

# OpenFlex MuJoCo English Guide

This project converts the OpenFlex v10 **full-body robot model** (mobile base + lift + dual arms +
head) — originally built for **ROS2 / RViz** — into a self-contained MJCF that can be opened by the
**plainest MuJoCo viewer**. On top of that it provides arm / head / lift control and gripper coupling.

Key idea: the "URDF → MuJoCo compile + inject actuators / gripper coupling" step is done **once** and
baked into a self-contained output XML. The viewers only load and display it — no runtime intermediate
files.

---

## 🖼️ Preview

![OpenFlex MuJoCo viewer 效果图](assets/1.png)

---

## 🚀 Quick Start

```bash
uv sync
# or: python3 -m pip install -r requirements.txt
```

```bash
python3 -c "import mujoco, mujoco.viewer, trimesh; print('MuJoCo environment OK')"
```

This repo **commits** `openflex_mujoco.xml` and `mujoco_meshes/`, so after cloning you can open the
model with the viewer **without running `convert.py`**:

```bash
# Plain clone — ready to view immediately (recommended, simplest)
git clone <this-repo-url> openflex_mujoco
cd openflex_mujoco
python3 viewer.py            # open the already-built output model
```

To rebuild from the **latest OpenFleX upstream models**, clone **recursively** to pull the 5 git
submodules under `packages/`, then re-run the compilation and conversion:

```bash
# Recursive clone: also fetches the 5 packages/ submodules (descriptions from OpenFleX-Wheeled-Humanoid org)
git clone --recursive <this-repo-url> openflex_mujoco
cd openflex_mujoco

# Pull submodules only (if you already cloned without --recursive):
git submodule update --init --recursive --remote

# Step 1: compile xacro → URDF (with latest joint definitions & parameters)
python3 build_urdf.py

# Step 2: URDF → MuJoCo (same as before)
python3 convert.py

# Step 3: view
python3 viewer.py
```

> - `git submodule update --init --recursive`: restores submodules to the commit recorded in the main repo (not necessarily latest)
> - `git submodule update --init --recursive --remote`: fetches and checks out the latest code from submodule remote
> - `build_urdf.py` compiles URDF from the xacro source in the `openarmx_integrated_description` submodule,
>   so joint connections and definitions always match the latest upstream (only needs `pip install xacro`, no full ROS2)
> - Without a recursive clone, `packages/` is empty and `convert.py` fails (no source meshes). In that
>   case just use the committed output XML + viewer (see "Plain clone" above).

Check only (no window):

```bash
python3 convert.py --check
python3 viewer.py  --check
```

---

## 💥 Two collision builds

The repo ships two output models, differing in whether the robot's own parts collide with
each other:

| Build | File | Description | Launch |
|---|---|---|---|
| A. Floor-only (default) | `openflex_mujoco.xml` | Robot collides only with the floor / external objects; **no self-collision** between links (avoids jitter from overlapping adjacent visual meshes) | `python3 viewer.py` |
| B. Full self-collision | `openflex_mujoco_selfcol.xml` | **arms / lift / body / chassis all collide with each other** (visual meshes collide directly, zero redundancy) | `python3 viewer.py --self-collision` |

How build B works (optimized, no redundancy):
- The visual mesh geoms themselves are made to collide: a MuJoCo mesh geom already renders AND
  collides; `contype/conaffinity` only decides *whether* it collides. So build B just flips the
  visual meshes' group from `2/1` (floor-only) to `3/3` (collide with everything). Each link uses
  **one geom for both rendering and collision** — no separate overlapping collision meshes needed.
- Because the chassis platform (base_link visual mesh) now collides too, the arm is stopped when
  pushed into the body / chassis.
- Adjacent parent-child bodies (joints) get `<exclude>` pairs to avoid jitter from overlapping
  geometry at the joint. Additionally, **all collisions between non-arm rigid bodies** are excluded:
  the chassis / wheels / lift column / body / head are one rigidly-mounted assembly with built-in
  mesh overlaps (4–9 cm); letting them collide would shove parts apart (the lift launching upward at
  startup). What remains is **every collision that involves an arm**: arm↔arm, arm↔body / chassis,
  arm↔lift column, arm↔head. The body keeps its collision volume (the arm can hit it); body parts
  just don't collide with each other.
- `convert.py` builds **both versions by default**.

> To use build B, just `python3 viewer.py --self-collision` after cloning (the output is committed);
> to rebuild from the latest OpenFleX models, run `git submodule update --init --recursive` first.

---

## 🗂️ Structure

```bash
.
├── build_urdf.py                    # Compiler: xacro → URDF (from latest submodule sources)
├── convert.py                       # Converter: URDF -> self-contained MJCF
├── viewer.py                        # Native MuJoCo viewer (loads the output XML)
├── openflex_integrated_robot.urdf   # Intermediate: compiled full-body URDF
├── packages/                        # Source: 5 git submodules (OpenFleX upstream descriptions, not in main history)
│   ├── openflex_chassis/            #   -> base_model_interface_layer (contains swerve_description)
│   ├── lift_slide_description/      #   -> OpenFleX-Wheeled-Humanoid/lift_slide_description
│   ├── openarmx_description/        #   -> OpenFleX-Wheeled-Humanoid/openarmx_description
│   ├── openarmx_head_description/   #   -> OpenFleX-Wheeled-Humanoid/openarmx_head_description
│   └── openarmx_integrated_description/  # -> integration xacro source (robot joint connections)
├── openflex_mujoco.xml              # Output (committed): self-contained MJCF (actuators / coupling / floor / lights)
└── mujoco_meshes/                   # Output (committed): converted MuJoCo-friendly meshes (.obj, relative paths)
```

`openflex_mujoco.xml` and `mujoco_meshes/` are **committed to the repo**, so a plain clone opens directly
in the viewer with no need to rerun `build_urdf.py` + `convert.py`; rebuild only when you want the
latest upstream models (requires a recursive clone of the `packages/` submodules).
`build_urdf.py` compiles URDF from the xacro source in the `openarmx_integrated_description` submodule,
ensuring joint connections always match the latest upstream.
The source description files in `packages/` are referenced as **git submodules** (from
`https://github.com/orgs/OpenFleX-Wheeled-Humanoid/repositories`) and are not stored in this repo's history,
keeping the main repository lightweight.

---

## 📘 Files

### `build_urdf.py`

Compiles `openflex_integrated_robot.urdf` from upstream xacro sources without a full ROS2 environment.

By injecting a fake `ament_index_python` module, `$(find ...)` in xacro resolves automatically to the local `packages/` submodules.
After upstream changes to joint connections or parameters, just `git submodule update --remote` + `python3 build_urdf.py` to get the latest URDF.

### `openflex_integrated_robot.urdf`
Full-body URDF (base + lift + dual arms + head), compiled by `build_urdf.py` from
`packages/openarmx_integrated_description/urdf/openarmx_integrated_robot.urdf.xacro`.
It references several `package://<pkg>/` meshes, which `convert.py` maps to the local `packages/<pkg>/` directory. Gripper
coupling uses `<mimic>`, which MuJoCo's URDF importer **silently drops**, so coupling is re-added
during conversion.

### `convert.py`
Converter only (no display):
1. Strips `<ros2_control>`; the URDF `<collision>` tags are dropped and the visual meshes double as
   collision geoms (collide with the floor / external objects, self-collision between links disabled to
   avoid jitter from overlapping adjacent visual meshes); adds default inertia to moving links that lack it
2. Converts every visual mesh (`.dae` / any `.stl` incl. ASCII) to `.obj` via trimesh (keeps colors, bakes negative scale)
3. Compiles to a MuJoCo model and serializes to MJCF
4. Merges floor + studio-style lights
5. Wraps the robot in a `yaw root` body (rotated about Z to face the default view)
6. Injects `<equality>` gripper coupling: each `finger_joint2` follows its `finger_joint1`
7. Injects position actuators: **`nu=23`** (14 arm joints + 2 master fingers + 2 head + 1 lift + 4 steering;
   continuous wheel joints get no actuator and rely on damping; `finger_joint2` is driven by the equality constraint)
8. Converges mesh paths into `mujoco_meshes/` as relative references, so the output XML is portable

The output `openflex_mujoco.xml` is self-contained and can be opened by any plain MuJoCo viewer, e.g.
`python -m mujoco.viewer openflex_mujoco.xml`.

### `viewer.py`
Native viewer: `mujoco.MjModel.from_xml_path(openflex_mujoco.xml)` + `mujoco.viewer.launch`.
Drag sliders on the right to control joints; drag with the mouse to orbit / zoom.

---

## 🤏 Gripper coupling

Each gripper has two prismatic joints: `finger_joint1` (axis `0 -1 0`) and `finger_joint2`
(axis `0 1 0`, `<mimic joint="finger_joint1"/>`), both range `0 0.044`. MuJoCo drops `<mimic>`,
so `convert.py` adds `<equality><joint joint1="finger_joint1" joint2="finger_joint2"/>`
(default `q_joint2 = q_joint1`). Because the axes are opposite, equal `q` yields a symmetric mirror
open/close, staying inside the `0 0.044` range. Only `finger_joint1` is exposed as a slider per side.

---

## ⚠️ FAQ

- **MuJoCo cannot find meshes**: run from the project root; if `packages/` or `mujoco_meshes/` is
  incomplete, rerun `python3 convert.py`.
- **Gripper won't move**: make sure you use `openflex_mujoco.xml` from `convert.py`. `convert.py --check`
  should report `nu=23` (14 arm + 2 master fingers + 2 head + 1 lift + 4 steering).
- **Wrong robot orientation**: change `ROBOT_YAW` in `convert.py` (`-1.57079632679`, i.e. -90° about Z).
- **Build B lift / body flying up or jittering**: build B already excludes **all collisions between
  non-arm rigid bodies** (chassis / wheels / lift column / body / head are one rigidly-mounted assembly
  with built-in overlaps that would otherwise be shoved apart, launching the lift upward at startup), so
  the lift / body are stable while arm collisions are kept. If you reintroduce this by editing
  `contype/conaffinity` or removing those `<exclude>` pairs, just rerun `python3 convert.py` to restore.
