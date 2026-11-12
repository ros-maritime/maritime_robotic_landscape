---
title: Quick reference
description: One-page cheat sheet for experienced developers - recommended packages, conventions, message types, and fast troubleshooting.
---

# Quick reference

A dense summary for experienced developers. Each section links to the full page.

## Default choices (October 2026)

| Need | Start with | Alternatives |
|---|---|---|
| Platform | ROS 2 Jazzy + Gazebo Harmonic, Ubuntu 24.04 | POSIM: ROS 2 Lyrical + Gazebo Jetty, Ubuntu 26.04 |
| UUV framework | Blue + auv_controllers | MVP, SMaRC 2 |
| BlueROV2 / ArduSub | ardusub_driver or MAVROS; POSIM for BlueROV2 + ArduSub SITL | Orca5 for visual odometry; Blue |
| ASV framework | Project11 | Nav2 + custom; MOOS-IvP |
| State estimation | robot_localization (two EKFs + navsat_transform) | fuse, GTSAM |
| Thrust allocation | Pseudo-inverse + scaling (thruster_manager, auv_controllers) | QP (mvp_control) |
| Surface simulation | VRX, POSIM (WAM-V + ArduRover SITL) | Stonefish, MARUS 2 |
| Underwater simulation | POSIM (DAVE); Gazebo Harmonic systems | Stonefish, HoloOcean, OceanSim |
| Sonar simulation | POSIM (DAVE); physics-based multibeam sonar | Stonefish, HoloOcean |
| Ready-to-launch application | WWW-POSIM (web and desktop, powered by POSIM) | |
| Messages | sensor_msgs, geometry_msgs, nav_msgs + marine_msgs | |
| Acoustic comms | ros_acomms (WHOI Micro-Modem) | Goby3 + DCCL |
| Logging | rosbag2 with MCAP | |

[Package catalog →](catalog.md)

## Conventions

- World **ENU**, body **FLU**, SI units, radians ([REP 103](https://www.ros.org/reps/rep-0103.html)). Convert NED and FRD at the driver.
- `map → odom → base_link → sensors` ([REP 105](https://www.ros.org/reps/rep-0105.html)). Controllers use `odom`; planners use `map`.
- Yaw (ENU) \( = \pi/2 - \) heading (NED). Roll keeps its sign, pitch changes sign.
- \( z = -\)depth. `FluidPressure` is in pascals.
- Stamp at the time of measurement, in ROS time.

[Frames and conventions →](conventions.md)

## Message types

| Data | Message |
|---|---|
| DVL velocity for fusion | `geometry_msgs/TwistWithCovarianceStamped` (raw: `marine_acoustic_msgs/Dvl`) |
| Attitude | `sensor_msgs/Imu` |
| Depth for fusion | `geometry_msgs/PoseWithCovarianceStamped` (z only) |
| Pressure | `sensor_msgs/FluidPressure` |
| GNSS | `sensor_msgs/NavSatFix` → navsat_transform |
| USBL / LBL fix | `geometry_msgs/PoseWithCovarianceStamped` in `map` |
| Imaging sonar | `marine_acoustic_msgs/ProjectedSonarImage` |
| Multibeam | `marine_acoustic_msgs/SonarDetections` + `sensor_msgs/PointCloud2` |
| Radar | `marine_sensor_msgs/RadarSector` |
| Control output | `geometry_msgs/Wrench` → allocator |

[Drivers and interfaces →](drivers.md)

## Fast troubleshooting

| Symptom | Check first | Then |
|---|---|---|
| EKF diverges or jumps | Timestamps (ROS time, time of measurement) | Covariances too small; NED published as ENU; broken TF |
| Vehicle turns the wrong way | Heading to yaw conversion; thruster direction signs in URDF | Allocation matrix moment arms |
| No DVL bottom lock | Altitude versus DVL range; soft or steep bottom | Acoustic interference; aerated water; thruster noise |
| Position drifts in a current | DVL in water-track mode | Lost bottom lock; heading bias |
| Cannot hold position | Buoyancy and trim; saturation | Allocation conditioning; thruster dead zones |
| Acoustic modem silent | Range test at the surface; frequency band and addresses | Thermocline, multipath, noise; transducer orientation |
| ROS 2 nodes not visible over radio | `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE` | DDS discovery over a slow link; use a bridge or rmw_zenoh |
| Simulation runs slowly | Real-time factor; sensor rates and image sizes | GPU drivers; headless rendering |

## Sim-to-real cautions

- Wave and current models are simplified; sensor noise is optimistic.
- Simulated sonar rarely models multipath and reverberation well.
- Retune controllers on hardware. Expect extra debugging time for the first sea trials.

## Field checklist (short)

- [ ] DVL and INS warm-up and alignment per manufacturer
- [ ] Depth sensor zeroed at the surface
- [ ] Acoustic range test at the surface before the dive
- [ ] All thrusters tested in both directions, vehicle restrained
- [ ] Abort and autonomous surfacing tested
- [ ] Logging running, disk space checked

[Full field operations guide →](operations.md)

## References

- T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.
- E. Gallimore, T. Schneider, H. Schmidt, "ROS Message Transport over Underwater Acoustic Links with ros_acomms," IEEE/OES AUV Symposium, 2022.
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization) · [ros2_control](https://control.ros.org/) · [Gazebo](https://gazebosim.org/) · [Nav2](https://docs.nav2.org/)
