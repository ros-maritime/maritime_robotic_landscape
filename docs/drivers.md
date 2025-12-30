# Drivers and Interfaces

This page catalogs ROS drivers for maritime sensors, actuators, and communication devices, plus the maritime_interfaces package for standard message definitions.

## Maritime Interfaces Package

**[maritime_interfaces](https://github.com/ros-maritime/maritime_interfaces)**

The ros-maritime organization maintains a package of standard message and service definitions for maritime robotics.

**Purpose:**
- Standardize data formats across maritime ROS packages
- Enable interoperability between different systems
- Provide maritime-specific message types

**Contents:**
- Sonar messages
- Acoustic communication messages
- Marine sensor messages
- Maritime-specific service definitions

**ROS Version:** ROS 2

### Other Standard Message Packages

#### marine_msgs

**[marine_msgs](https://github.com/apl-ocean-engineering/marine_msgs)** | University of Washington Applied Physics Lab

Standardized ROS message definitions for maritime robotics, providing common data types across the community.

**Features:**
- USBL position messages
- DVL velocity and bottom track
- Maritime navigation data types
- Acoustic communication messages
- ROS 1 and ROS 2 support

**Status:** Actively maintained with ROS 2 support

**Why Use It:** Provides community-standard message definitions that promote interoperability between different maritime robotics projects.

#### marine_acoustic_msgs

Message definitions specifically for acoustic sensors and communication in marine environments. (No current public repository link available.)

## Sonar Drivers

### Imaging Sonar

**[ping360_sonar](https://github.com/CentraleNantesRobotics/ping360_sonar)**
- **Manufacturer:** Blue Robotics Ping360
- **Type:** Mechanically scanning imaging sonar
- **ROS Version:** ROS 1
- **Features:**
  - Point cloud generation
  - Configurable range and gain
  - Angular resolution control
  - Image and scan data output

### Multibeam Sonar

Various manufacturers provide multibeam sonar systems. Drivers are often manufacturer-specific or use generic interfaces:

- Teledyne BlueView series
- Oculus multibeam imaging sonar
- Nortek multibeam systems

**Integration Approaches:**
- Manufacturer SDKs wrapped in ROS nodes
- Generic multibeam interfaces
- Point cloud output for SLAM integration

### Forward-Looking Sonar (FLS)

Used for obstacle avoidance and navigation:
- Short-range detection
- Real-time obstacle mapping
- Integration with autonomy systems

## Navigation Sensor Drivers

### DVL (Doppler Velocity Log)

Essential for underwater navigation.

**Common Manufacturers:**
- **Teledyne RDI:** Workhorse, Explorer DVL
- **Nortek:** DVL1000, DVL500
- **LinkQuest:** NavQuest series
- **Water Linked:** DVL A50

⚠️ **Note:** The [waterlinked/dvl-a50-ros-driver](https://github.com/waterlinked/dvl-a50-ros-driver) GitHub repository is archived (last push 2024-02-26).

**ROS Integration:**
- Velocity measurements (body frame)
- Bottom-lock altitude
- Water-track velocity
- Integration with robot_localization
- See [whoi_ds](#whoi-deep-submergence-lab-drivers) for WHOI DVL drivers

### IMU / INS Drivers

**[sbg_ros2_driver](https://github.com/SBG-Systems/sbg_ros2_driver)**
- **Manufacturer:** SBG Systems (Ellipse series)
- **ROS Version:** ROS 2
- **Features:**
  - Orientation (quaternion, Euler angles)
  - Angular velocity
  - Linear acceleration
  - Magnetometer data
  - GNSS integration (for surface)
  - Time synchronization

**Other IMU Drivers:**
- VectorNav ROS drivers (community-developed)
- Xsens drivers
- LORD MicroStrain drivers
- Standard sensor_msgs/Imu compatible sensors

### Pressure / Depth Sensors

**Integration:**
- Simple serial or I2C interface
- Published as sensor_msgs/FluidPressure
- Depth calculation from pressure
- Temperature compensation

## Acoustic Communication

### WHOI ros_acomms ⭐

**[ros_acomms](https://git.whoi.edu/acomms/ros_acomms)** | [Documentation](https://acomms.pages.whoi.edu/ros_acomms/overview.html)

Woods Hole Oceanographic Institution's ROS-based acoustic communication stack. See the project documentation for deployment history and supported configurations.

**Features:**
- Transport ROS messages over underwater acoustic links
- Modular modem driver architecture
- Message queue management for link optimization
- Media Access Control (MAC) engines
- Acoustic link simulator with ray-tracing model

**Supported Hardware:**
- WHOI Micro-Modem family (primary target)
- Iridium satellite links (low-throughput mode)
- Extensible to other acoustic modems via modular driver interface

**ROS Version:** ROS 1 (ROS 2 migration status unknown)

**Why Use It:** ros_acomms provides an end-to-end acoustic communications stack for ROS; see the documentation for feature and configuration details.

See the [Software page](software.md#whoi-ros_acomms) for details on features, applications, and performance characteristics.

### Manufacturer-Specific Drivers

Various acoustic modem manufacturers provide or support ROS integration:
- EvoLogics modems
- Teledyne Benthos
- Sonardyne

## GNSS / GPS Drivers

For surface vehicles:

**Standard Drivers:**
- [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver): NMEA 0183 protocol support (ROS 1)
- [nmea_gps_driver](https://github.com/ros-drivers/nmea_gps_driver): NMEA support for ROS 2
- u-blox drivers for u-blox receivers
- SwiftNav Piksi drivers

**Use Cases:**
- Surface vehicle positioning
- Time synchronization
- Heading (with dual GPS)

## Environmental Sensors

### CTD (Conductivity, Temperature, Depth)

**Integration:**
- Serial interface (most common)
- Custom message types or sensor_msgs
- Logging for oceanographic data

**Manufacturers:**
- Sea-Bird Scientific
- RBR
- Aanderaa

### Weather Stations

For surface vessels:
- Wind speed and direction
- Air temperature and pressure
- Humidity
- Integration with meteorological data systems

## WHOI Deep Submergence Lab Drivers

### whoi_ds

**[whoi_ds](https://bitbucket.org/whoidsl/workspace/repositories/)** | WHOI Bitbucket workspace (repo access may require approval)

Woods Hole Oceanographic Institution Deep Submergence Lab's ROS driver package for deep submergence vehicle sensors.

**Status:** ROS 1 (legacy package, part of WHOI DSL ecosystem)

**Supported Sensors:**

**Navigation Sensors:**
- **DVL Drivers:** Teledyne RDI DVLs (Workhorse, Explorer series)
  - Bottom-track and water-track velocity
  - Altitude measurements
  - Beam diagnostics
- **INS/IMU:** Kearfott INS, PHINS INS, other inertial navigation systems
- **USBL:** Underwater acoustic positioning systems

**Environmental Sensors:**
- **CTD:** Sea-Bird SBE 49 FastCAT and other CTD sensors
- **Altimeters:** Teledyne Benthos acoustic altimeters
- **Depth Sensors:** Paroscientific pressure sensors

**Acoustic Systems:**
- **Sonars:** Integration with various imaging and profiling sonars
- **Acoustic Modems:** Integration layer for underwater communication devices

**Message Definitions:**
The package includes `ds_msgs` message definitions for deep submergence sensors, providing standardized interfaces for the WHOI sensor ecosystem.

**Integration:**
- Works with the WHOI DSL simulation environment ([ds_sim](https://bitbucket.org/whoidsl/ds_sim))
- Designed for deep submergence vehicle operations
- Provides standardized interfaces for multi-sensor fusion

**Note:** For new projects, consider whether modern ROS 2 alternatives are available. This package represents field-tested code from deep submergence operations (see repository history for details).

## Actuator Drivers

### Thruster Controllers

**Integration Approaches:**

1. **Direct PWM Control:**
   - ROS node generates PWM signals
   - Hardware interface (e.g., PCA9685)
   - Simple, low-level control

2. **Through Flight Controller:**
   - PX4, ArduSub integration
   - MAVLink protocol
   - Higher-level interface

3. **ros2_control Integration:**
   - Hardware abstraction
   - Controller lifecycle management
   - Standardized interfaces

**Common Hardware:**
- Blue Robotics ESCs (Electronic Speed Controllers)
- Custom motor controllers
- Servo-based control surfaces

### Manipulator Control

For underwater manipulators:
- Joint position controllers
- Gripper control
- Force/torque sensing
- Integration with MoveIt for motion planning

## Camera Drivers

### Underwater Cameras

**Standard Drivers:**
- [usb_cam](http://wiki.ros.org/usb_cam): USB camera support (ROS 1)
- [ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera): Video4Linux cameras (ROS 2)
- gscam: GStreamer-based camera pipeline

**Specialized:**
- Deep-sea cameras with serial control
- Stereo camera systems
- Low-light / scientific cameras

**Considerations:**
- Color correction for underwater
- Lighting control
- Synchronization for stereo

### Topside Cameras

Surface vehicles may use:
- Standard computer vision cameras
- Thermal cameras (FLIR drivers)
- PTZ (Pan-Tilt-Zoom) cameras

## Communication Hardware

### Serial Devices

Most marine sensors use serial communication:

**ROS Integration:**
- [serial](http://wiki.ros.org/serial) library
- Custom serial node development
- rosserial for microcontrollers

### Ethernet Devices

Many modern sensors use Ethernet:
- TCP/IP communication
- UDP for real-time data
- Web interfaces for configuration

## Power Monitoring

Battery and power system monitoring:

**Measurements:**
- Voltage
- Current
- State of Charge (SoC)
- Temperature

**Integration:**
- Battery Management System (BMS) interfaces
- Custom power monitoring nodes
- Safety cutoff logic

## Multi-Sensor Integration

### Sensor Fusion Frameworks

**[robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)**
- Fuses multiple sensors (IMU, GPS, DVL, etc.)
- EKF and UKF implementations
- Standard sensor_msgs inputs
- Handles discontinuous measurements (e.g., acoustic positioning)

## Driver Development Guidelines

When developing new maritime sensor drivers:

1. **Use Standard Messages:** Prefer sensor_msgs and nav_msgs where possible
2. **Consider maritime_interfaces:** Use or contribute to maritime-specific message types
3. **Time Synchronization:** Properly timestamp all sensor data
4. **Configuration:** Use ROS parameters for sensor configuration
5. **Error Handling:** Report sensor faults and connection issues
6. **Documentation:** Provide clear documentation for setup and configuration
7. **ROS 2:** Target ROS 2 for new development when possible

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
