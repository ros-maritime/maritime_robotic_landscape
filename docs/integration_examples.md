# Integration Examples

**For experienced maritime roboticists.** Practical reference architectures, launch file patterns, and integration configurations from proven deployments.

!!! warning "Adapt to Your System"

    The launch files and configurations shown here are **illustrative examples** based on common integration patterns. You MUST adapt them to your specific:

    - Hardware configuration (sensor models, frame IDs, ports)
    - Vehicle dynamics (process noise, allocation matrices)
    - Mission requirements (update rates, sensor priorities)

    Test thoroughly in simulation before deploying to hardware. Incorrect parameters can cause instability or damage.

---

## Reference Architectures

### Torpedo AUV (Survey Vehicle)

**Typical configuration:** Forward-only propulsion, surveying seafloor at constant altitude.

**Sensor Suite:**
- DVL (bottom-track velocity + altitude)
- INS/IMU (orientation, angular velocity)
- Depth sensor (pressure-based)
- GPS (surface positioning)
- Optional: Sidescan sonar, cameras, CTD

**Software Stack:**
```
GPS (surface) ──┐
DVL (velocity)  ├──> robot_localization (EKF) ──> nav_msgs/Odometry
IMU (attitude)  ─┤
Depth (z)    ────┘

Odometry ──> Mission Planner ──> PID Controller ──> Thruster Commands
```

**Key Integration Points:**
- EKF fuses DVL, IMU, depth continuously
- GPS updates only at surface (discontinuous)
- Mission planner uses fused odometry for waypoint following
- Single thruster allocation (1-DOF control in surge)

### Hovering AUV/ROV (Inspection Platform)

**Typical configuration:** 6-DOF control, station-keeping for close-range inspection.

**Sensor Suite:**
- DVL (3-axis velocity)
- INS (6-DOF orientation)
- Depth sensor
- Cameras (mono or stereo)
- Lights
- Forward-looking sonar (obstacle detection)
- Optional: USBL beacon, manipulator

**Software Stack:**
```
DVL ──────┐
INS ──────┼──> robot_localization (EKF) ──> Odometry
Depth ────┤
USBL ─────┘ (low rate)

Odometry ──> 6-DOF Controller ──> Thrust Allocation ──> Thrusters

Cameras ──> Vision Pipeline ──> Object Detection ──> Mission Supervisor
FLS ─────> Obstacle Detection ─────────────────────┘
```

**Key Integration Points:**
- Fuse DVL/INS at the native sensor rates
- USBL updates are low-rate and need outlier rejection
- Thrust allocation handles multi-thruster configurations
- Mission supervisor coordinates inspection behaviors

### ASV (Surface Survey)

**Typical configuration:** Differential drive or twin-thruster, autonomous surface mapping.

**Sensor Suite:**
- GPS (primary positioning)
- IMU (heading, orientation)
- Optional: DVL (current measurement), sonar, cameras

**Software Stack:**
```
GPS ──────┐
IMU ──────┼──> robot_localization (EKF) ──> Odometry
DVL ──────┘ (if available)

Odometry ──> Nav2 ──> Differential Drive Controller ──> Thrusters
                │
Sonar/Camera ───> Costmap ──> Path Planning
```

**Key Integration Points:**
- Nav2 provides path planning and obstacle avoidance
- GPS sufficient for positioning (no DVL required)
- Wave/current compensation via DVL or modeling
- Costmap integration for dynamic obstacles

---

## Launch File Patterns

### Basic Sensor Fusion (robot_localization)

**File:** `config/ekf.yaml` (example structure)

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: <imu_rate>  # Set to your IMU rate
    sensor_timeout: <sensor_timeout>  # Set to your sensor cadence
    two_d_mode: false

    # Frame configuration
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # DVL: Velocity in body frame
    odom0: /dvl/velocity
    odom0_config: [false, false, false,    # x, y, z position (DVL doesn't provide absolute position)
                   false, false, false,    # roll, pitch, yaw
                   true,  true,  true,     # x_dot, y_dot, z_dot (body-frame velocities)
                   false, false, false,    # roll_dot, pitch_dot, yaw_dot
                   false, false, false]    # x_ddot, y_ddot, z_ddot
    odom0_differential: false
    odom0_relative: false

    # IMU: Orientation and angular velocity
    imu0: /imu/data
    imu0_config: [false, false, false,     # x, y, z position
                  true,  true,  true,      # roll, pitch, yaw (orientation)
                  false, false, false,     # velocities
                  true,  true,  true,      # roll_dot, pitch_dot, yaw_dot (angular velocities)
                  false, false, false]     # accelerations
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    # Depth sensor: Absolute z position
    pose0: /depth/pose
    pose0_config: [false, false, true,     # x, y, z (only z from depth)
                   false, false, false,    # orientations
                   false, false, false,    # velocities
                   false, false, false,    # angular velocities
                   false, false, false]    # accelerations
    pose0_differential: false

    # GPS (surface only): Absolute x, y position
    # Only enable when surfaced
    pose1: /gps/fix
    pose1_config: [true,  true,  false,    # x, y (lat/lon converted)
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]

    # Process noise covariances: see robot_localization docs and tune for your vehicle
```

**Launch File:** `launch/localization.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_auv_package')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # DVL driver
        Node(
            package='your_dvl_driver',
            executable='dvl_node',
            name='dvl',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'dvl_link'}],
            remappings=[('velocity', '/dvl/velocity')]
        ),

        # IMU driver
        Node(
            package='your_imu_driver',
            executable='imu_node',
            name='imu',
            parameters=[os.path.join(pkg_share, 'config', 'imu.yaml')],
            remappings=[('imu/data', '/imu/data')]
        ),

        # Depth sensor
        Node(
            package='your_depth_driver',
            executable='depth_node',
            name='depth',
            parameters=[{'port': '/dev/ttyUSB1', 'frame_id': 'depth_link'}],
            remappings=[('depth', '/depth/pose')]
        ),

        # EKF localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odometry/local')]
        ),
    ])
```

### Thrust Allocation (6-DOF ROV)

**File:** `config/thruster_allocation.yaml`

```yaml
thruster_manager:
  ros__parameters:
    # Thruster configuration matrix
    # Row 1-3: Linear forces (surge, sway, heave)
    # Row 4-6: Moments (roll, pitch, yaw)
    # Columns: One per thruster

    allocation_matrix: [
      # Fill with your vehicle's geometry and thrust directions.
    ]

    # Thruster limits
    max_thrust: <max_thrust>  # Set to your thruster limit
    min_thrust: <min_thrust>  # Set to your thruster limit

    # Saturation handling
    use_saturation: true
    saturation_method: "quadratic_programming"  # or "pseudo_inverse"
```

**Launch with MVP Control (QP-based):**

```python
Node(
    package='mvp_control',
    executable='thruster_allocator',
    name='thruster_allocator',
    parameters=[
        {'config_file': os.path.join(pkg_share, 'config', 'thruster_config.yaml')}
    ],
    remappings=[
        ('desired_forces', '/control/wrench'),
        ('thruster_forces', '/thrusters/commands')
    ]
)
```

### Complete AUV Launch Example (Skeleton)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_auv')

    return LaunchDescription([
        # ============ SENSORS ============
        # DVL
        Node(
            package='your_dvl_driver',
            executable='dvl_node',
            name='dvl',
            parameters=[os.path.join(pkg_share, 'config', 'dvl.yaml')],
        ),

        # INS
        Node(
            package='your_ins_driver',
            executable='ins_node',
            name='ins',
            parameters=[os.path.join(pkg_share, 'config', 'ins.yaml')],
        ),

        # Depth
        Node(
            package='your_depth_driver',
            executable='depth_node',
            name='depth',
        ),

        # ============ LOCALIZATION ============
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[os.path.join(pkg_share, 'config', 'ekf_local.yaml')],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),

        # ============ CONTROL ============
        # PID Controllers
        Node(
            package='your_controller_pkg',
            executable='controller_node',
            name='depth_controller',
            parameters=[os.path.join(pkg_share, 'config', 'depth_pid.yaml')],
            remappings=[
                ('state', '/odometry/local'),
                ('setpoint', '/control/depth_setpoint'),
                ('control_effort', '/control/depth_effort')
            ],
        ),

        Node(
            package='your_controller_pkg',
            executable='controller_node',
            name='heading_controller',
            parameters=[os.path.join(pkg_share, 'config', 'heading_pid.yaml')],
            remappings=[
                ('state', '/odometry/local'),
                ('setpoint', '/control/heading_setpoint'),
                ('control_effort', '/control/heading_effort')
            ],
        ),

        # Thrust allocation
        Node(
            package='your_control_pkg',
            executable='thruster_allocator',
            name='thrust_allocator',
            parameters=[os.path.join(pkg_share, 'config', 'thrusters.yaml')],
        ),

        # ============ AUTONOMY ============
        Node(
            package='my_auv',
            executable='mission_manager',
            name='mission_manager',
            parameters=[os.path.join(pkg_share, 'config', 'mission.yaml')],
        ),

        # ============ ACTUATORS ============
        Node(
            package='my_auv',
            executable='thruster_driver',
            name='thrusters',
            parameters=[{'serial_port': '/dev/ttyACM0'}],
        ),

        # ============ STATIC TRANSFORMS ============
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_dvl',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dvl_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_ins',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ins_link']
        ),
    ])
```

---

## TF Tree Configurations

### Standard Torpedo AUV

```
map (world-fixed, global reference, GPS-aligned)
 └─ odom (continuous but drifting, dead reckoning origin)
     └─ base_link (robot body center, origin of robot frame)
         ├─ dvl_link (DVL sensor, offset forward and down)
         ├─ ins_link (INS sensor, typically at CoG)
         ├─ depth_link (depth sensor location)
         ├─ camera_link (forward camera)
         ├─ sonar_link (sidescan or multibeam)
         └─ thruster_link (propeller location)
```

**Key transforms:**
- `map → odom`: Updated by GPS fixes (discontinuous, only at surface)
- `odom → base_link`: Published by robot_localization (continuous, rate matches your configuration)
- `base_link → sensors`: Static transforms, defined in URDF or static_transform_publisher

**Configuration:**
```xml
<!-- URDF snippet -->
<joint name="base_to_dvl" type="fixed">
  <parent link="base_link"/>
  <child link="dvl_link"/>
  <origin xyz="0.15 0 -0.10" rpy="0 0 0"/>
</joint>

<joint name="base_to_ins" type="fixed">
  <parent link="base_link"/>
  <child link="ins_link"/>
  <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
</joint>
```

**Launch:**
```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': urdf_content}]
)
```

### Hovering ROV with Manipulator

```
world (simulator or map frame)
 └─ odom
     └─ base_link
         ├─ dvl_link
         ├─ ins_link
         ├─ camera_front_link
         ├─ camera_down_link
         ├─ lights_link
         ├─ thruster_1_link
         ├─ thruster_2_link
         │  ... (additional thrusters as needed)
         └─ manipulator_base_link
             └─ manipulator_link_1
                 └─ manipulator_link_2
                     └─ gripper_link
```

**Dynamic transforms:**
- Manipulator joints published by `joint_state_publisher` or `ros2_control`
- MoveIt uses these for inverse kinematics

### ASV (Surface Vehicle)

```
map (UTM or local grid)
 └─ odom
     └─ base_link
         ├─ gps_link (antenna location, offset from CoG)
         ├─ imu_link (typically at base_link)
         ├─ camera_link
         ├─ lidar_link (if equipped)
         └─ sonar_link (multibeam or sidescan)
```

**GPS offset critical:** GPS antenna not at robot center affects heading calculation in dual-GPS setups.

---

## Message Flow Patterns

### High-Rate Sensor Fusion

**Sensor rates:**
- Use the native sensor rates from your device manuals and driver settings

**EKF configuration:**
```yaml
# High-frequency prediction from IMU
frequency: <imu_rate>  # Set to IMU rate

# DVL corrections at your configured DVL rate
# EKF interpolates between measurements

# USBL corrections are low rate
# Use large covariance, enable outlier rejection
```

**Timing critical:**
- All sensors must have synchronized timestamps
- Use `message_filters::TimeSynchronizer` if needed for multi-sensor processing
- EKF handles asynchronous measurements internally

### Control Loop Architecture

**Cascaded control (common pattern):**

```
Mission Planner (mission cadence)
    ↓ (desired waypoint)
Path Follower (path cadence)
    ↓ (desired velocity/heading)
Velocity Controller (controller cadence)
    ↓ (desired forces/torques)
Thrust Allocator (allocator cadence)
    ↓ (individual thruster commands)
Thruster Driver (driver cadence)
```

**Message types:**
```
nav_msgs/Path → geometry_msgs/Twist → geometry_msgs/Wrench → std_msgs/Float64MultiArray
```

**Timing considerations:**
- Control loop must run faster than vehicle dynamics
- Allocator should match control rate
- Actuator commands can be higher rate (reduces jitter)

### Acoustic Communication Pattern

**Challenge:** Very low bandwidth and high latency

**Message prioritization:**
```python
class AcousticPublisher:
    def __init__(self):
        # High priority: Emergency commands, status
        self.emergency_queue = PriorityQueue()

        # Medium priority: Mission updates
        self.mission_queue = Queue()

        # Low priority: Diagnostics, logs
        self.diagnostic_queue = Queue()

    def send_next(self):
        if not self.emergency_queue.empty():
            return self.emergency_queue.get()
        elif not self.mission_queue.empty():
            return self.mission_queue.get()
        else:
            return self.diagnostic_queue.get()
```

**ros_acomms integration:**
```python
# Vehicle side
Node(
    package='ros_acomms',
    executable='modem_driver',
    parameters=[{
        'modem_id': 1,
        'rate': 0,  # Placeholder; set per modem specification
        'vehicle_config': 'modem_config.yaml'
    }],
    remappings=[
        ('rx', '/acomms/received'),
        ('tx', '/acomms/transmit')
    ]
)

# Topside
Node(
    package='ros_acomms',
    executable='modem_driver',
    parameters=[{
        'modem_id': 2,
        'rate': 0  # Placeholder; set per modem specification
    }]
)
```

---

## Sensor Suite Timing

### Recommended Synchronization

**Method 1: Hardware timestamping**
- Use PPS (Pulse Per Second) from GPS
- Distribute to all sensors via trigger line
- Sensors timestamp data with hardware clock

**Method 2: Software timestamping**
- NTP or PTP time synchronization
- Sensors use system time
- Less accurate but simpler

**ROS 2 timing:**
```python
# In sensor driver
msg.header.stamp = self.get_clock().now().to_msg()

# EKF uses message timestamps
# Ensure sensor drivers use ROS time, not wall time
```

### Multi-Sensor Synchronization Example

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

class MultiSensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to multiple sensors
        image_sub = Subscriber(self, Image, '/camera/image')
        sonar_sub = Subscriber(self, SonarImage, '/sonar/image')
        odom_sub = Subscriber(self, Odometry, '/odometry/local')

        # Synchronize within a configurable window
        sync = ApproximateTimeSynchronizer(
            [image_sub, sonar_sub, odom_sub],
            queue_size=10,
            slop=<sync_tolerance_s>  # Set to your tolerance
        )
        sync.registerCallback(self.sync_callback)

    def sync_callback(self, image_msg, sonar_msg, odom_msg):
        # Messages arrive within the configured tolerance
        # Process fused data
        pass
```

---

## Troubleshooting Integration Issues

### TF Tree Debugging

**Problem:** "Could not find a connection between 'map' and 'base_link'"

**Debug:**
```bash
# View full TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link

# Monitor TF rates
ros2 topic hz /tf
ros2 topic hz /tf_static
```

**Common causes:**
- Missing static transform
- Parent frame not published
- Circular dependency

### EKF Divergence

**Symptoms:**
- Position estimate jumps
- Covariance grows unbounded
- Innovation values very large

**Debug:**
```bash
# Monitor EKF diagnostics
ros2 topic echo /diagnostics

# Check sensor rates
ros2 topic hz /dvl/velocity
ros2 topic hz /imu/data

# Verify timestamps
ros2 topic echo /dvl/velocity --field header.stamp
```

**Common fixes:**
- Reduce sensor covariance (too confident in bad data)
- Increase process noise (model doesn't match reality)
- Check for timestamp errors (using wall time instead of ROS time)
- Disable problematic sensor temporarily

### Thrust Allocation Saturation

**Symptoms:**
- Vehicle can't maintain position
- Constant thruster saturation warnings
- Poor trajectory following

**Debug:**
```python
# Monitor allocation output
ros2 topic echo /thruster_allocator/status

# Check desired vs. achievable forces
ros2 topic echo /control/wrench
ros2 topic echo /thrusters/forces
```

**Solutions:**
- Reduce commanded velocities
- Improve vehicle buoyancy (reduce constant heave thrust requirement)
- Check allocation matrix condition number
- Add more thrusters or reposition existing ones

---

## References

* [robot_localization documentation](https://docs.ros.org/en/melodic/api/robot_localization/html/)
* [ros2_control documentation](https://control.ros.org/)
* [Nav2 documentation](https://docs.nav2.org/)
* [MVP project examples](https://github.com/uri-ocean-robotics/mvp_readme)
* [Blue project launch files](https://github.com/Robotic-Decision-Making-Lab/blue)

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
