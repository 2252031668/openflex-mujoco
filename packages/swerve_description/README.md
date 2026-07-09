# swerve_description

English | [中文](./README-CN.md)

---

URDF model and visualization configuration for the 4-wheel-steer 4-wheel-drive (4WS4WD) swerve chassis.

## Description

This package contains the complete robot description (URDF/Xacro) for the OpenArmX swerve drive chassis, including:

- Base frame (base_link) with 3D mesh
- 4 swerve modules (FL, FR, BL, BR) each with a steering joint and a drive wheel joint
- Livox MID-360 LiDAR mount (mid360_link, livox_frame)
- Intel RealSense D435 camera mount (d435_link, camera_link, depth/color optical frames)
- IMU frame (imu_link, co-located with MID-360)
- GPS frame (gps_link)
- ros2_control hardware interface configuration (swerve_ros2_control.urdf.xacro)
- RViz visualization config

## Chassis Specifications

| Parameter | Value |
|-----------|-------|
| Wheel radius | 0.075 m |
| Wheelbase (front-rear) | 0.42 m |
| Track width (left-right) | 0.547 m |
| Steering motors | RS06 (CAN5, extended frame, position mode) |
| Drive motors | UM series hub motors (CAN4, CANopen CiA402 PV mode) |
| Steering range | +/-90 degrees (+/-1.5708 rad) |

## TF Tree

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

## Xacro Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gazebo` | `false` | Reserved for simulation compatibility |
| `use_mock` | `false` | Use fake_components/GenericSystem instead of real hardware |
| `steering_can_interface` | `can5` | CAN bus for RS06 steering motors |
| `driving_can_interface` | `can4` | CAN bus for UM drive motors |
| `controllers_file` | `""` | Controllers YAML path |

## Configuration

Chassis kinematic parameters are defined in `config/chassis_version_6.0.yaml` and serve as the single source of truth for all downstream launch files and controllers.

## Build

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select swerve_description
source install/setup.bash
```

## Dependencies

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`

## License

Apache-2.0
