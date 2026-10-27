---
title: Drivers and interfaces
description: ROS 2 driver status for marine navigation sensors, sonars, modems, and cameras, plus the marine_msgs message definitions.
---

# Drivers and interfaces

This page answers two questions: *does my sensor have a maintained ROS 2 driver?* and *which message type should it publish?*

## ROS 2 driver status by vendor

Checked in October 2026 against each vendor's public repositories. "Official" means the vendor publishes or maintains the driver.

| Sensor | Vendor / product | ROS 2 driver | Source |
|---|---|---|---|
| DVL | Water Linked A50, A125 | :material-check-circle:{ .mrl-yes title="Yes" } Official | [waterlinked_dvl](https://github.com/waterlinked/waterlinked_dvl) (humble, jazzy branches) |
| DVL | Nortek Nucleus | :material-check-circle:{ .mrl-yes title="Yes" } Official | [nucleus_driver](https://github.com/nortekgroup/nucleus_driver) |
| DVL | Teledyne RDI (Workhorse, Pathfinder, Tasman) | :material-minus-circle-outline:{ .mrl-no title="Community only" } None found from vendor | Community and lab drivers; WHOI DSL has ROS 1 drivers |
| INS / IMU | SBG Systems | :material-check-circle:{ .mrl-yes title="Yes" } Official, binary release | [sbg_ros2_driver](https://github.com/SBG-Systems/sbg_ros2_driver) |
| INS / IMU | MicroStrain by HBK | :material-check-circle:{ .mrl-yes title="Yes" } Official, binary release | [microstrain_inertial](https://github.com/LORD-MicroStrain/microstrain_inertial) |
| INS / IMU | Xsens (Movella) MTi | :material-check-circle:{ .mrl-yes title="Yes" } Vendor support repository | [Xsens MTi ROS driver](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client) |
| INS / IMU | Advanced Navigation | :material-check-circle:{ .mrl-yes title="Yes" } Vendor example driver (Humble) | [ros2-driver](https://github.com/advanced-navigation/ros2-driver) |
| INS / IMU | VectorNav | :material-account-group-outline:{ .mrl-partial title="Community" } Community | [dawonn/vectornav](https://github.com/dawonn/vectornav) (ros2 branch) |
| INS | Exail (PHINS), Kearfott, Sonardyne SPRINT | :material-minus-circle-outline:{ .mrl-no title="None public" } None public | Usually NMEA or vendor protocols over serial or Ethernet |
| GNSS | Any NMEA 0183 receiver | :material-check-circle:{ .mrl-yes title="Yes" } Binary release | [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) (ros2 branch) |
| GNSS | u-blox F9P and others | :material-check-circle:{ .mrl-yes title="Yes" } Community, binary release | [ublox_dgnss](https://github.com/aussierobots/ublox_dgnss), [ublox](https://github.com/KumarRobotics/ublox) |
| GNSS | Swift Navigation | :material-check-circle:{ .mrl-yes title="Yes" } Official | [swiftnav-ros2](https://github.com/swift-nav/swiftnav-ros2) |
| Imaging sonar | Blue Robotics Ping360 | :material-account-group-outline:{ .mrl-partial title="Community" } Community (no official ROS 2 driver) | [ping360_sonar](https://github.com/CentraleNantesRobotics/ping360_sonar), [bluerobotics_ping360](https://github.com/GSO-soslab/bluerobotics_ping360), [blueos-ros2](https://github.com/itskalvik/blueos-ros2) |
| Echosounder | Blue Robotics Ping2 | :material-account-group-outline:{ .mrl-partial title="Community" } Community | [blueos-ros2](https://github.com/itskalvik/blueos-ros2); build on [ping-python](https://github.com/bluerobotics/ping-python) |
| Imaging sonar | Blueprint Subsea Oculus | :material-account-group-outline:{ .mrl-partial title="Community" } Community | [blueprint_oculus_sonar](https://github.com/GSO-soslab/blueprint_oculus_sonar), [oculus_ros2](https://github.com/ENSTABretagneRobotics/oculus_ros2), [liboculus](https://github.com/apl-ocean-engineering/liboculus), [oculus-m3000d-ros2-cpp-driver](https://github.com/AlePuglisi/oculus-m3000d-ros2-cpp-driver) |
| Multibeam | Cerulean Surveyor 240-16 | :material-account-group-outline:{ .mrl-partial title="Community" } Community | [cerulean_surveyor_mbes](https://github.com/GSO-soslab/cerulean_surveyor_mbes) |
| Sonar | Tritech, Imagenex, Teledyne BlueView, Kongsberg | :material-minus-circle-outline:{ .mrl-no title="None public" } No maintained public driver found | Use the vendor SDK in your own node |
| Acoustic modem | WHOI Micro-Modem | :material-progress-check:{ .mrl-partial title="In progress" } ROS 1, ROS 2 support in progress | [ros_acomms](https://acomms.pages.whoi.edu/ros_acomms/overview.html) |
| Acoustic modem | EvoLogics, Sonardyne, Teledyne Benthos | :material-minus-circle-outline:{ .mrl-no title="None public" } None found | Only an old ROS 1 EvoLogics driver exists |
| Autopilot | ArduSub, ArduPilot Rover, PX4 | :material-check-circle:{ .mrl-yes title="Yes" } Binary release | [mavros](https://github.com/mavlink/mavros), [ardusub_driver](https://github.com/Robotic-Decision-Making-Lab/ardusub_driver), ArduPilot and PX4 DDS interfaces |
| Camera | USB, V4L2, GStreamer streams | :material-check-circle:{ .mrl-yes title="Yes" } Binary release | [usb_cam](https://github.com/ros-drivers/usb_cam), [gscam](https://github.com/ros-drivers/gscam) |

If your sensor has no driver, see [Write your own driver](#write-your-own-driver) below. Most marine sensors speak NMEA 0183 or a simple binary protocol over serial or UDP.

## Navigation sensors

<!-- catalog:table category=drivers -->

## Sonar

<!-- catalog:table category=sonar -->

## Cameras, lidar, and AIS

<!-- catalog:table category=sensors -->

## Message definitions

Use standard ROS messages where they fit, and the community **marine_msgs** set for sonar, DVL, and radar data. Do not invent a new message type if one of these fits.

<!-- catalog:table category=interfaces -->

### Which message for which sensor

| Sensor | Recommended message | Notes |
|---|---|---|
| IMU / AHRS | `sensor_msgs/Imu` | Orientation in ENU and FLU per REP 103 and REP 145. Many marine INS units output NED; convert in the driver. |
| DVL velocity | `marine_acoustic_msgs/Dvl` for raw data; `geometry_msgs/TwistWithCovarianceStamped` for fusion | robot_localization takes a twist input. Set `frame_id` to the DVL frame and publish its mounting transform. |
| DVL altitude / single-beam echosounder | `sensor_msgs/Range` | |
| Depth / pressure | `sensor_msgs/FluidPressure` (pascals) | Convert to depth in a separate node, and publish depth as a `geometry_msgs/PoseWithCovarianceStamped` with only z set for the EKF. |
| GNSS | `sensor_msgs/NavSatFix` | Fuse it through `navsat_transform_node`, not directly. |
| USBL / LBL fix | `geometry_msgs/PoseWithCovarianceStamped` or `geographic_msgs/GeoPoseStamped` | No dedicated community message exists yet. Give a realistic covariance. |
| Imaging sonar | `marine_acoustic_msgs/ProjectedSonarImage` or `RawSonarImage` | sonar_image_proc can display and convert these. |
| Multibeam / profiling sonar | `marine_acoustic_msgs/SonarDetections` or `SonarRanges` | Also publish `sensor_msgs/PointCloud2` for generic tools. |
| Radar | `marine_sensor_msgs/RadarSector`, `RadarEcho` | |
| CTD, turbidity | `marine_sensor_msgs/TurbidityNTU`; CTD has no standard message yet | Use `sensor_msgs/Temperature` and `FluidPressure` for parts of a CTD. |
| AIS | Messages from [marine_ais](https://github.com/rolker/marine_ais) | |

See [Frames and conventions](conventions.md) for frame IDs, units, and sign conventions.

## WHOI Deep Submergence Lab software

The WHOI Deep Submergence Lab (DSL) maintains ROS 1 drivers and tools for deep submergence vehicles in its [Bitbucket workspace](https://bitbucket.org/whoidsl/workspace/repositories/). This includes drivers for Teledyne RDI DVLs, inertial navigation systems, CTDs, altimeters, and the `ds_msgs` message set, plus the [ds_sim](https://bitbucket.org/whoidsl/ds_sim) Gazebo plugins. Some repositories need access approval. The software comes from many years of field use on vehicles such as Sentry and Jason, and is a useful reference when you write a ROS 2 driver for the same instruments.

## Write your own driver

Many marine instruments have no ROS 2 driver. A good driver is small and predictable:

1. **Separate the protocol from ROS.** Put parsing in a plain library with unit tests, and keep the ROS node thin. This also lets you reuse the parser in post-processing tools.
2. **Use a reusable transport.** [transport_drivers](https://github.com/ros-drivers/transport_drivers) gives serial and UDP nodes for ROS 2.
3. **Use standard messages** from the table above, and SI units (REP 103).
4. **Timestamp correctly.** Stamp each message with the time of measurement, not the time of publication. If the device gives a hardware time, map it to ROS time and document how. See [time synchronization](conventions.md#time).
5. **Convert frames in the driver.** Publish ENU and FLU data, and state in the README when the device outputs NED or FRD.
6. **Report health.** Publish `diagnostic_msgs/DiagnosticArray` (for example through `diagnostic_updater`) with connection state, data rate, and device status flags such as DVL bottom lock.
7. **Make it configurable.** Expose port, baud rate, frame ID, and rates as ROS parameters, and supply a launch file and an example configuration.
8. **Release it.** Add CI ([action-ros-ci](https://github.com/ros-tooling/action-ros-ci) or [industrial_ci](https://github.com/ros-industrial/industrial_ci)) and release the package to the ROS build farm so others can `apt install` it. Then add it to the [catalog](catalog.md).
