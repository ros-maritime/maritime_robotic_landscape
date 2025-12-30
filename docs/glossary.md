# Glossary

A beginner-friendly guide to maritime robotics terminology. Terms are organized alphabetically within categories for easy reference.

## Core Concepts

**Marine Robotics / Maritime Robotics**
: Building and operating robots that work in, on, or underwater environments.

**ROS (Robot Operating System)**
: Open-source software framework for building robot applications. Despite the name, it's not an operating system like Windows or Linux - it's a set of tools and libraries that run on top of an OS (usually Linux). Think of it as a toolkit that helps robots' different parts talk to each other.

**ROS 2**
: The current generation of ROS with improved real-time behavior and security features.

**Autonomy / Autonomous**
: The robot can make decisions and operate on its own without constant human control. An autonomous robot can navigate, avoid obstacles, and complete missions independently.

**Simulation / Simulator**
: Software that creates a virtual environment where you can test robots without needing physical hardware or water. Like a video game for robots - you can make mistakes safely and test ideas quickly.

## Vehicle Types

### Surface Vehicles

**ASV (Autonomous Surface Vehicle)**
: A boat or ship that operates on the water's surface without a crew onboard. It makes its own navigation decisions.

**USV (Uncrewed Surface Vehicle)**
: Similar to ASV - a surface vessel with no people aboard. The terms are often used interchangeably, though USV can include both autonomous and remotely-controlled vessels.

**WAM-V**
: Wave Adaptive Modular Vessel - a specific type of catamaran (twin-hull boat) platform used in robotics competitions and research.

### Underwater Vehicles

**AUV (Autonomous Underwater Vehicle)**
: A robot submarine that operates independently underwater. No cable to the surface - it follows pre-programmed missions or makes its own decisions.

**ROV (Remotely Operated Vehicle)**
: An underwater robot controlled by a human operator via a tether (cable). The cable provides power and communication. Like flying a drone, but underwater.

**UUV (Unmanned Underwater Vehicle)**
: General term for any underwater vehicle without people inside - includes both AUVs and ROVs.

**Glider**
: A special type of AUV that doesn't have a propeller. It uses buoyancy changes to glide through the water in a sawtooth pattern. Energy-efficient for long missions.

**Torpedo-shaped AUV**
: An AUV with a streamlined, cylindrical hull (like a torpedo). Efficient for long-distance travel in a straight line.

**Hovering AUV**
: An AUV with multiple thrusters allowing it to move in any direction and hover in place. Good for detailed inspection work.

## Sensors and Navigation

**DVL (Doppler Velocity Log)**
: A sonar-based sensor that measures how fast the robot is moving relative to the seafloor or water. Essential for underwater navigation since GPS doesn't work underwater. Uses the Doppler effect (same physics that makes ambulance sirens change pitch as they pass you).

**IMU (Inertial Measurement Unit)**
: A sensor that measures acceleration and rotation. Helps the robot know which way is up and how it's oriented. Contains accelerometers and gyroscopes.

**INS (Inertial Navigation System)**
: A more advanced version of an IMU that can calculate position by tracking all movements. Higher accuracy than a basic IMU.

**GPS / GNSS (Global Navigation Satellite System)**
: Satellite-based positioning system. **Important:** GPS doesn't work underwater! Only useful when a vehicle surfaces. GNSS is the generic term for all satellite navigation systems (GPS, GLONASS, Galileo, etc.).

**Depth Sensor / Pressure Sensor**
: Measures water pressure to determine depth underwater. Deeper water = higher pressure. Simple and reliable.

**CTD (Conductivity, Temperature, Depth)**
: A sensor package that measures three key ocean properties:
  - **Conductivity:** How well water conducts electricity (indicates salinity/saltiness)
  - **Temperature:** Water temperature
  - **Depth:** How deep the robot is

**Sonar**
: Uses sound waves to "see" underwater (like bat echolocation). Light doesn't travel far in water, but sound does. Different types:
  - **Imaging Sonar:** Creates pictures from sound reflections
  - **Multibeam Sonar:** Maps the seafloor
  - **Side-scan Sonar:** Creates wide-area images of the seabed
  - **Forward-looking Sonar (FLS):** Detects obstacles ahead

**Hydrophone**
: An underwater microphone. Listens to sounds in the ocean.

## Control and Motion

**DOF (Degrees of Freedom)**
: The number of independent directions a robot can move. A car has 2 DOF (forward/backward and turn). Marine robots typically have 6 DOF:
  - **Surge:** Forward/backward
  - **Sway:** Left/right (sideways)
  - **Heave:** Up/down
  - **Roll:** Rotation around front-to-back axis (like a barrel roll)
  - **Pitch:** Rotation around left-to-right axis (nose up/down)
  - **Yaw:** Rotation around vertical axis (turning left/right)

**Thruster**
: An underwater propeller that pushes the robot. Most marine robots have multiple thrusters to control different directions.

**Thrust Allocation**
: The math that figures out how much power to send to each thruster to make the robot move in the desired direction.

**Azimuth Thruster**
: A thruster that can rotate to point in different directions. More flexible than fixed thrusters.

**Buoyancy**
: The upward force that makes things float. Submarines and AUVs carefully balance their buoyancy to neither sink nor float.

**Ballast**
: Weight added to a robot to control buoyancy. Some robots can pump water in/out to adjust their ballast.

## Control Systems

**PID Controller**
: A common control algorithm (Proportional-Integral-Derivative). Helps the robot smoothly reach a target position or heading. Like cruise control in a car.

**Waypoint**
: A target position or GPS coordinate the robot should navigate to. A mission might have many waypoints in sequence.

**Path Planning**
: Algorithm that calculates the route the robot should take from start to finish, avoiding obstacles.

**Localization**
: Figuring out where the robot is. Underwater this is hard since GPS doesn't work - robots use DVL, IMU, and sometimes acoustic beacons.

**SLAM (Simultaneous Localization and Mapping)**
: Building a map of an unknown environment while simultaneously figuring out where you are in that map. Challenging but powerful technique.

**EKF / UKF (Extended/Unscented Kalman Filter)**
: Mathematical techniques for combining data from multiple sensors to get a better estimate of position and velocity. Handles noisy sensor data intelligently.

## Communication

**Acoustic Communication / Acoustic Modem**
: Underwater "wireless" communication using sound waves. Much slower than WiFi (much lower data rates) but works underwater where radio doesn't.

**WHOI Micro-Modem**
: A widely-used underwater acoustic modem developed by Woods Hole Oceanographic Institution. Industry standard for underwater communication.

**RF (Radio Frequency)**
: Traditional wireless communication (like WiFi, cellular, radio). Works great in air, but doesn't work underwater - water absorbs radio waves quickly.

**Tether**
: The cable connecting an ROV to the surface. Provides power and high-bandwidth communication (like an underwater Ethernet cable). Limits how far the ROV can go.

**Satellite Communication**
: Communication via satellites. Used by surface vehicles or AUVs when they surface. Expensive but works anywhere on Earth.

**USBL (Ultra-Short Baseline)**
: Acoustic positioning system that uses sound to track an underwater robot's position from a surface ship. Measures range and bearing (direction).

**LBL (Long Baseline)**
: A network of underwater acoustic beacons deployed on the seafloor. The robot triangulates its position using these beacons. More accurate than USBL but requires setup.

## Software and Development

**Gazebo**
: A widely used robotics simulator. Can simulate physics, sensors, and robot movement in a 3D environment. Free and open-source.

**VRX (Virtual RobotX)**
: A simulated maritime robotics competition based on Gazebo.

**DAVE (Dave Aquatic Virtual Environment)**
: A simulation environment for underwater vehicles, built on Gazebo.

**RViz**
: A visualization tool for ROS. Shows you what the robot "sees" - sensor data, camera images, point clouds, etc. Helpful for debugging.

**Topic**
: In ROS, a named channel where messages are published. For example, `/camera/image` might be the topic where camera images are published. Like a TV channel - anyone can tune in.

**Node**
: In ROS, a single program or process that does one specific job. A robot might have many nodes working together (one for camera, one for navigation, one for control, etc.).

**Message**
: In ROS, data sent over a topic. Has a specific format/structure. Like a package being delivered.

**Launch File**
: A file that starts multiple ROS nodes at once with their configurations. Convenient for starting complex robot systems.

**Colcon / Catkin**
: Build tools for ROS. Colcon is used in ROS 2, Catkin was used in ROS 1. They compile your code and set up the workspace.

## Organizations and Institutions

**WHOI (Woods Hole Oceanographic Institution)**
: A major ocean research institution in Massachusetts, USA, with a long history in marine robotics and ocean engineering.

**MBARI (Monterey Bay Aquarium Research Institute)**
: Ocean research institute in California with significant work in marine robotics.

**ROS-Maritime**
: The working group focused on marine robotics within the larger ROS community. Maintains packages, documentation, and hosts meetings.

## Competition and Standards

**RobotX**
: An international maritime robotics competition for autonomous surface vehicles. Teams compete in tasks like navigation, object detection, and docking.

**MBZIRC (Mohamed Bin Zayed International Robotics Challenge)**
: A major robotics competition that includes maritime challenges.

**SAUVC (Singapore AUV Challenge)**
: Student-focused underwater robotics competition. Held in Singapore.

**MATE ROV Competition**
: Popular student competition for remotely operated vehicles. Multiple divisions from high school to university.

## Coordinate Frames

**NED (North-East-Down)**
: A coordinate system where:
  - X points North
  - Y points East
  - Z points Down (toward Earth's center)
  - Common in aerospace and marine navigation

**ENU (East-North-Up)**
: A coordinate system where:
  - X points East
  - Y points North
  - Z points Up (away from Earth)
  - Common in robotics and computer graphics

**Body Frame**
: A coordinate system attached to the robot. X typically points forward, Y to the right, Z down. Moves and rotates with the robot.

## Miscellaneous Terms

**Open-source**
: Software where the code is freely available for anyone to use, modify, and share. ROS and most tools in this landscape are open source.

**URDF (Unified Robot Description Format)**
: An XML format for describing a robot's physical structure (links, joints, sensors). Tells ROS what your robot looks like.

**TF / TF2 (Transform)**
: ROS system for tracking coordinate frames. Keeps track of where different parts of the robot are relative to each other and the world.

**Workspace**
: In ROS, a directory containing your code and packages. Where you do development.

**Package**
: In ROS, a collection of related code, configuration files, and documentation organized together. Like an app or library.

**Repository / Repo**
: A place where code is stored and version-controlled, usually on GitHub or GitLab.

**CI (Continuous Integration)**
: Automated testing that runs every time code changes. Catches bugs early.

---

## Need More Help?

- **Can't find a term?** Ask on [ROS Discourse](https://discourse.ros.org/c/maritime/36)
- **Want deeper explanation?** See [Education & Tutorials](education_and_tutorials.md)
- **Just starting?** Check the [Getting Started Guide](getting_started.md)

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
