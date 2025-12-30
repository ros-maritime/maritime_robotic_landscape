# Maritime Vehicle Types

Maritime robotics encompasses a diverse range of vehicle platforms, each designed for specific environments and missions. This page provides an overview of the main vehicle categories used in ROS-based maritime robotics.

## Surface Vehicles (ASV/USV)

Autonomous Surface Vehicles (ASVs) and Uncrewed Surface Vehicles (USVs) operate on the water's surface. These platforms are commonly used for:

- Hydrographic surveying and mapping
- Environmental monitoring
- Autonomous cargo transport
- Search and rescue operations
- Oceanographic data collection

**Characteristics:**
- Degrees of Freedom: Typically 3-4 DOF (surge, sway, yaw, sometimes heave)
- Typical Sensors: GPS/GNSS, IMU, cameras, radar, lidar, weather stations, AIS
- Propulsion: Propellers, azimuth thrusters, water jets, sail (for autonomous sailboats)
- Power: Batteries, solar panels, diesel generators, hybrid systems
- Communication: RF, cellular, satellite

**Common Platforms:**
- Autonomous survey vessels
- Wave energy converters (WEC)
- Autonomous sailboats
- Multi-hull vessels (catamarans, trimarans)

## Underwater Vehicles

### Autonomous Underwater Vehicles (AUVs)

AUVs are untethered robotic submarines that operate independently underwater. They come in two main configurations:

#### Torpedo-Shaped AUVs
- **Design:** Streamlined, cylindrical hull optimized for forward motion
- **Propulsion:** Rear-mounted propeller(s)
- **Use Cases:** Long-range missions, oceanographic surveys, pipeline inspection
- **DOF:** Primarily 6-DOF but optimized for forward motion

#### Hovering AUVs
- **Design:** More compact, multi-thruster configuration
- **Propulsion:** Multiple thrusters for omnidirectional movement
- **Use Cases:** Detailed inspection, intervention tasks, confined spaces
- **DOF:** Full 6-DOF (surge, sway, heave, roll, pitch, yaw)

**Typical Sensors:**
- Doppler Velocity Log (DVL) for velocity and altitude
- Inertial Navigation System (INS) / IMU
- Depth sensor / pressure transducer
- Sonar (imaging, multibeam, side-scan, forward-looking)
- Cameras (visible light, low-light)
- CTD (Conductivity, Temperature, Depth)
- Hydrophones

### Remotely Operated Vehicles (ROVs)

ROVs are tethered underwater robots controlled by operators on the surface.

**Characteristics:**
- **Tether:** Physical cable for power and communication
- **Control:** Real-time operator control with varying levels of automation
- **DOF:** Typically 4-6 DOF depending on thruster configuration
- **Power:** Supplied via tether (mission duration limited by surface support)
- **Use Cases:** Deep-sea exploration, underwater construction, ship hull inspection, scientific research

**Common Configurations:**
- Observation-class ROVs: Lightweight, for inspection
- Work-class ROVs: Heavy-duty, with manipulators
- Hybrid ROVs/AUVs: Can operate in both modes

### Underwater Gliders

Gliders are a special class of AUV that use buoyancy control for propulsion.

**Characteristics:**
- **Propulsion:** Buoyancy engine (no propeller)
- **Motion:** Sawtooth pattern through water column
- **Endurance:** Long-duration missions, depending on vehicle design
- **Speed:** Slow relative to propeller-driven vehicles
- **Use Cases:** Long-duration oceanographic monitoring, large-scale surveys
- **Sensors:** Typically CTD, oxygen sensors, fluorometers, acoustic sensors

## Specialized Platforms

### Autonomous Sailboats

Unmanned sailing vessels using wind power for propulsion.

- **Propulsion:** Wind-powered sail with autonomous control
- **Endurance:** Long endurance with proper design
- **Use Cases:** Ocean monitoring, long-duration surveys, racing competitions
- **Control Challenges:** Wind prediction, sail trim optimization, path planning

### Intervention Vehicles

Underwater vehicles equipped with manipulators for interaction tasks.

- **Manipulators:** Robotic arms (typically 1-2 per vehicle)
- **End Effectors:** Grippers, cutters, sensors, tools
- **Use Cases:** Valve turning, sample collection, cable laying, underwater construction

### Bio-Inspired Vehicles

Vehicles that mimic marine animal locomotion.

- **Biomimetic fish:** Fin-based propulsion
- **Turtle robots:** Flipper-based propulsion
- **Jellyfish robots:** Pulsed propulsion
- **Use Cases:** Research, stealth operations, efficient low-speed propulsion

## Vehicle Comparison

Compare vehicles using measurable, documented factors from manufacturers and field reports:

- Operating environment and depth rating
- Mission duration and energy source
- Required degrees of freedom and maneuverability
- Communications constraints
- Payload capacity and integration needs

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
