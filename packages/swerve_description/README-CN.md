# swerve_description

[English](./README.md) | 中文

---

四转四驱（4WS4WD）舵轮底盘 URDF 模型与可视化配置。

## 简介

本包包含 OpenArmX 舵轮底盘的完整机器人描述（URDF/Xacro），包括：

- 底座框架（base_link）含 3D 网格模型
- 4 个舵轮模块（FL、FR、BL、BR），各含转向关节和驱动轮关节
- Livox MID-360 激光雷达安装（mid360_link、livox_frame）
- Intel RealSense D435 相机安装（d435_link、camera_link、深度/彩色光学坐标系）
- IMU 坐标系（imu_link，与 MID-360 同位置）
- GPS 坐标系（gps_link）
- ros2_control 硬件接口配置（swerve_ros2_control.urdf.xacro）
- RViz 可视化配置

## 底盘参数

| 参数 | 值 |
|------|-----|
| 车轮半径 | 0.075 m |
| 轴距（前后） | 0.42 m |
| 轮距（左右） | 0.547 m |
| 转向电机 | 灵足时代 RS06（CAN5，扩展帧，位置模式） |
| 驱动电机 | 和利时 UM 系列伺服轮毂一体机（CAN4，CANopen CiA402 PV 模式） |
| 转向范围 | +/-90 度（+/-1.5708 rad） |

## TF 树

```
base_link
├── base_footprint
├── fl_steering_link → fl_wheel_link
├── fr_steering_link → fr_wheel_link
├── bl_steering_link → bl_wheel_link
├── br_steering_link → br_wheel_link
├── mid360_link → livox_frame
│              → imu_link
├── d435_link → camera_link → camera_depth_frame → camera_depth_optical_frame
│            │              → camera_rgb_frame → camera_rgb_optical_frame
│            → d435_depth_frame → d435_depth_optical_frame
│            → d435_color_frame → d435_color_optical_frame
└── gps_link
```

## Xacro 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_gazebo` | `false` | 保留参数，兼容仿真 |
| `use_mock` | `false` | 使用 fake_components/GenericSystem 替代真实硬件 |
| `steering_can_interface` | `can5` | RS06 转向电机 CAN 总线 |
| `driving_can_interface` | `can4` | UM 驱动电机 CAN 总线 |
| `controllers_file` | `""` | 控制器 YAML 路径 |

## 配置

底盘运动学参数定义在 `config/chassis_version_6.0.yaml` 中，作为所有下游 launch 文件和控制器的唯一参数来源。

## 编译

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select swerve_description
source install/setup.bash
```

## 依赖

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`

## 许可证

Apache-2.0
