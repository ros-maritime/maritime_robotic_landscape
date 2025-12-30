# Education and Tutorials

Resources for learning maritime robotics with ROS, from beginner tutorials to advanced topics.

!!! note "Absolute Beginner?"

    **Never used ROS before?** Start with the [Getting Started Guide](getting_started.md) for a complete beginner introduction. The tutorials on this page assume you have basic familiarity with ROS concepts.

    **Need to understand the terms?** Check the [Glossary](glossary.md) for explanations of acronyms and concepts.

## Getting Started with Maritime Robotics

### Introduction to Marine Vehicles

**Prerequisites:**
- **Basic ROS knowledge** - Understand topics, nodes, and launch files
- **Basic Linux/command-line** - Can navigate directories, edit files
- **Python or C++** - Know basic programming concepts

**New to ROS?** Start with:
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html) - Official beginner tutorials
- [ROS 2 Basics](https://www.ros.org/) - Understand what ROS is
- [Getting Started Guide](getting_started.md) - Maritime robotics introduction

**Maritime-Specific Concepts:**
1. **Vehicle Dynamics:** Understanding marine vehicle motion (6-DOF)
2. **Coordinate Frames:** NED (North-East-Down), ENU (East-North-Up), body frame
3. **Environmental Forces:** Currents, waves, buoyancy, drag
4. **Sensor Limitations:** GPS unavailable underwater, acoustic vs. RF communication

### Recommended Learning Path

1. **Simulation First:** Start with simulators (VRX, DAVE) before real hardware
2. **Simple Vehicles:** Begin with surface vehicles (easier to test and debug)
3. **Sensor Integration:** Add sensors incrementally (IMU → GPS → Sonar)
4. **Control Basics:** Understand PID control before advanced techniques
5. **Real Hardware:** Progress to physical platforms after simulation success

## Tutorials

### Tutorial 1: Setting Up VRX Simulation

**Goal:** Get the Virtual RobotX simulation running with a basic surface vehicle.

**Steps:**

1. **Follow the VRX README:** Install dependencies, build the workspace, and launch a sample world using the current VRX instructions.
2. **Confirm the vehicle spawns:** Ensure you see the model in the simulator before proceeding.
3. **Publish simple commands:** Use `ros2 topic pub` to command the thrusters (see the VRX docs for topic names and message types).

**Next Steps:**
- Explore sensor topics with `ros2 topic list`
- Write a simple controller node
- Try competition tasks

**Resources:**
- [VRX Wiki](https://github.com/osrf/vrx/wiki)
- [VRX Tutorials](https://github.com/osrf/vrx/wiki/tutorials)

### Tutorial 2: Understanding Thrust Allocation

**Goal:** Learn how multi-thruster vehicles distribute forces.

**Concept:**

A vehicle with multiple thrusters needs to determine how to command each thruster to achieve desired forces and torques. This is the thrust allocation problem.

**Example - 4-Thruster Configuration:**

Consider a 4-thruster configuration with horizontal thrusters:

```
     T1    T2
     ↓      ↓
    ┌────────┐
    │  ROV   │
    └────────┘
     ↑      ↑
     T3    T4
```

**Allocation Matrix T:**

Each thruster contributes to surge (X), sway (Y), and yaw (N). The allocation matrix relates thruster forces to vehicle forces:

```
[X]   [T_matrix] [F1]
[Y] = [  ....  ] [F2]
[N]   [  ....  ] [F3]
                 [F4]
```

**Implementation:**

1. **Define Configuration in URDF** (see thrust allocation packages)
2. **Use an existing package** (MVP-Control, thruster_manager)
3. **Or implement your own:**

```python
import numpy as np

# Pseudo-inverse solution
T = np.array([[1, 1, 1, 1],      # Surge
              [0, 0, 0, 0],       # Sway (simplified)
              [0.1, -0.1, -0.1, 0.1]])  # Yaw (moment arms)

T_inv = np.linalg.pinv(T)  # Pseudo-inverse

desired_forces = np.array([10, 0, 1])  # Example values
thruster_forces = T_inv @ desired_forces
```

**Gotchas:**
- Thruster saturation
- Dead zones
- Optimal vs. feasible solutions

### Tutorial 3: Integrating a DVL with ROS 2

**Goal:** Set up a Doppler Velocity Log for underwater navigation.

**Hardware:** Various DVL manufacturers (Teledyne RDI, Nortek, LinkQuest)

⚠️ **Note:** The Water Linked DVL A50 ROS driver repository ([waterlinked/dvl-a50-ros-driver](https://github.com/waterlinked/dvl-a50-ros-driver)) is archived (last push 2024-02-26). For current DVL driver options, see the [WHOI Deep Submergence Lab drivers](drivers.md#whoi-deep-submergence-lab-drivers) which support Teledyne RDI DVLs, or check the [Drivers page](drivers.md#dvl-doppler-velocity-log) for other options.

**Steps:**

1. **Install Driver:**
   - Clone the appropriate DVL driver for your hardware.
   - For WHOI drivers, see the WHOI Bitbucket workspace (repo access may require approval): https://bitbucket.org/whoidsl/workspace/repositories/

2. **Configure Connection:**
   Edit the config file for your serial port, baud rate, and frame ID per the device manual.

3. **Launch the DVL node:**
   Use your driver package's launch file.

4. **Verify Data:**
   Use `ros2 topic echo` to confirm the driver publishes velocity and altitude.

5. **Integrate with robot_localization:**

   ```yaml
   # ekf_config.yaml
   ekf_filter_node:
     ros__parameters:
       odom0: /dvl/velocity
       odom0_config: [false, false, false,  # x, y, z position
                      false, false, false,  # roll, pitch, yaw
                      true,  true,  true,   # x_dot, y_dot, z_dot
                      false, false, false,  # roll_dot, pitch_dot, yaw_dot
                      false, false, false]  # x_ddot, y_ddot, z_ddot
   ```

**Next Steps:**
- Add IMU for full state estimation
- Tune EKF parameters
- Test in water tank or simulation

### Tutorial 4: Basic Acoustic Localization

**Goal:** Understand USBL positioning for underwater vehicles.

**Concept:**

USBL (Ultra-Short Baseline) uses a surface transceiver to track an underwater beacon:
- Measures range (time-of-flight)
- Measures bearing (phase difference on transducer array)
- Computes 3D position relative to surface unit

**Simulation Setup:**

1. Use a simulator with acoustic positioning (DAVE, custom Gazebo)
2. Configure USBL beacon on vehicle
3. Configure surface transceiver

**ROS Integration:**

```python
# Simplified USBL position update
class USBLLocalizer:
    def __init__(self):
        self.sub = self.create_subscription(
            USBLFix, '/usbl/fix', self.usbl_callback, 10)

    def usbl_callback(self, msg):
        # msg contains: range, bearing, elevation
        # Convert to Cartesian coordinates
        x = msg.range * cos(msg.elevation) * cos(msg.bearing)
        y = msg.range * cos(msg.elevation) * sin(msg.bearing)
        z = msg.range * sin(msg.elevation)

        # Publish as NavSatFix or PoseWithCovariance
        # Integrate with EKF
```

**Challenges:**
- Outlier rejection (multipath, noise)
- Time synchronization
- Covariance estimation

## Workshops and Courses

### REMARO Summer School

**[REMARO Underwater Robotics Hackathon](https://github.com/remaro-network/tudelft_hackathon)**

Educational materials from TU Delft underwater robotics workshop.

**Contents:**
- Hands-on exercises
- Simulation scenarios
- Team challenges
- Real robot demonstrations

**Topics Covered:**
- AUV basics
- Sensor integration
- Path planning
- Object detection

### Academic Courses

Universities offering maritime robotics courses (often with public materials):

* **MIT:** Autonomous Marine Vehicles
* **NTNU (Norway):** Marine Cybernetics
* **University of Porto:** Underwater Robotics
* **TU Delft:** Marine Robotics

**Many publish:**
- Lecture slides
- Lab assignments
- Project descriptions
- Code repositories

## Documentation Resources

### Key Package Documentation

* **[robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html):** Sensor fusion for navigation
* **[ros2_control](https://control.ros.org/master/index.html):** Controller framework
* **[MoveIt 2](https://moveit.ros.org/):** Motion planning (for manipulators)
* **[Navigation2](https://docs.nav2.org/):** Path planning and navigation stack

### Maritime-Specific Docs

* **VRX Wiki:** Virtual RobotX competition and simulation
* **UUV Simulator Docs:** Classic underwater simulator (ROS 1)
* **DAVE Wiki:** DAVE simulator documentation

## Video Tutorials

### YouTube Channels

Channels with maritime robotics content:

* **WHOI Video:** Research vessel and AUV operations, deep submergence vehicle deployments
* **Blue Robotics:** ROV setup and operation, component tutorials
* **ROS Developers:** General ROS tutorials applicable to maritime

### WHOI Deep Submergence Lab Resources

Woods Hole Oceanographic Institution has been a pioneer in underwater robotics for decades. Their Deep Submergence Lab has contributed significantly to the field through:

**Historical Contributions:**
- Development of iconic vehicles: Alvin, Jason, ABE, Sentry
- Pioneering acoustic communication and positioning systems
- Advanced sensor integration for deep ocean exploration
- Real-world operational experience from thousands of dives

**Educational Impact:**
- [ros_acomms](https://git.whoi.edu/acomms/ros_acomms): Acoustic communication stack with extensive documentation
- [whoi_ds](https://bitbucket.org/whoidsl/workspace/repositories/): Sensor driver package (repo access may require approval)
- [ds_sim](https://github.com/Field-Robotics-Lab/ds_sim): Simulation environment for deep submergence vehicles
- Field-tested code from real ocean deployments

**Why Learn from WHOI:**
- **Proven in Practice:** Code has thousands of hours of operational sea time
- **Real-World Focus:** Addresses actual challenges faced in ocean robotics
- **Comprehensive Documentation:** Built for operational use, not just research
- **Industry Standard:** WHOI Micro-Modem and acoustic communication protocols are widely adopted

**Learning Resources:**
- [WHOI GitLab](https://git.whoi.edu/): Active development repositories
- [WHOI Bitbucket](https://bitbucket.org/whoidsl/workspace/repositories/): Legacy packages and deep submergence lab code
- Research publications documenting vehicle operations and sensor systems

### Conference Presentations

* **OCEANS Conference:** Technical presentations on marine robotics
* **ICRA/IROS:** Robotics conferences with maritime tracks
* **ROSCon:** ROS conference with maritime applications

## Books and Publications

### Recommended Books

* **"Handbook of Marine Craft Hydrodynamics and Motion Control"** by Thor I. Fossen
  * Definitive text on marine vehicle dynamics
  * Mathematical foundations for control

* **"Underwater Robotics: Science, Design & Fabrication"** by Steven W. Moore
  * Practical guide to ROV design
  * Hands-on projects

* **"Autonomous Underwater Vehicles"** by Gwyn Griffiths
  * AUV systems and design
  * Operational considerations

### Key Papers

* Control allocation methods
* Marine SLAM techniques
* Underwater communication protocols
* Path planning for marine vehicles

## Community Resources

### Forums and Discussion

* **ROS Discourse - Maritime Category:** Ask questions, share projects
* **Matrix Chat:** Real-time discussion with community members
* **GitHub Discussions:** Package-specific questions

### Example Projects

Learning from others' work:

* Browse repositories in ros-maritime organization
* Study competition code (VRX participants)
* Academic research code releases

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
