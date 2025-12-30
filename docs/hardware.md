# Hardware, Platforms, and Components

This page covers commercial and research platforms, development kits, sensors, and components for maritime robotics.

## Commercial UUV Platforms

### BlueROV2

The BlueROV2 is a commercial ROV platform used in research and education.

**Specifications:**
- **Type:** ROV (can be converted to AUV with additional hardware)
- **Depth Rating:** Varies by configuration; see vendor specifications
- **Thrusters:** Multi-thruster configuration (vendor-specific)
- **Payload:** Cameras, sonar, manipulator options
- **Power:** Tethered

**ROS Support:**
- Community-maintained Gazebo models and ROS drivers are available; verify compatibility for your ROS version

**Manufacturer:** Blue Robotics

### LRAUV-Based Systems

Long-range AUVs based on MBARI's open designs. See MBARI documentation for capabilities and sensor suites.

## Commercial USV/ASV Platforms

Commercial autonomous surface vehicles are available from various manufacturers:

* **Survey Vessels:** Hydrographic mapping-focused platforms
* **Multi-purpose ASVs:** Configurable for various missions
* **Research Platforms:** Universities and labs maintain custom ASV designs

**Key Suppliers:**
- Maritime Robotics (Mariner, Otter)
- ASV Global
- L3Harris (C-Worker series)
- Ocean Aero (Submaran S10)

## Development Kits

### Research Platforms

* **University Custom Designs:** Many labs build custom platforms
* **Open-source Designs:** Community-shared hull designs and electronics

## Sensors

### Navigation Sensors

#### Doppler Velocity Log (DVL)

Essential for underwater navigation, DVLs measure velocity relative to the seafloor or water column.

**Manufacturers:**
- Teledyne RDI
- Nortek
- LinkQuest
- Water Linked (note: DVL-A50 driver repo is archived)

⚠️ **Driver Note:** The [waterlinked/dvl-a50-ros-driver](https://github.com/waterlinked/dvl-a50-ros-driver) GitHub repository is archived (last push 2024-02-26). For DVL driver options, see the [WHOI Deep Submergence Lab drivers](drivers.md#whoi-deep-submergence-lab-drivers).

**Key Features:**
- Bottom-track and water-track modes
- Altitude measurement
- Beam configurations and accuracy depend on model (see datasheets)

**ROS Integration:** See [Drivers page](drivers.md#dvl-doppler-velocity-log) for available ROS drivers

#### DVL Selection Guide

Choosing the right DVL depends on mission requirements, budget, and operational environment.

**Comparison Matrix:**

| Vendor | Model | Spec Sheet |
|--------|-------|------------|
| **Teledyne RDI** | [Tasman DVL](https://www.teledynemarine.com/en-us/products/SiteAssets/RD%20Instruments/Tasman_DVL.pdf) | Teledyne Marine datasheet |
| **Nortek** | [DVL1000](https://www.nortekgroup.com/products/dvl-1000-300m) | [DVL1000 datasheet](https://www.nortekgroup.com/export/pdf/DVL1000%20-%204000%20m.pdf) |
| **Nortek** | [DVL500](https://www.nortekgroup.com/products/dvl500-300-m) | Nortek product page |
| **Water Linked** | [DVL A50](https://waterlinked.com/dvl/) | Water Linked product page |

**Decision Criteria:**

1. **Budget-Constrained:**
   - ⚠️ Water Linked DVL A50 driver repo is archived; confirm driver support before selecting
   - Consider used DVLs or phased procurement
   - Alternative: Dead reckoning with high-quality INS only (limited duration)

2. **Shallow Water Research:**
   - Prioritize compact, lower-power DVLs with reliable bottom-track

3. **Deep Ocean:**
   - Prioritize depth-rated systems with strong vendor support

4. **Long-Range AUV:**
   - Prioritize low power consumption and extended bottom-track range

5. **High-Accuracy Survey:**
   - Prioritize models with published accuracy specs and calibration procedures

**Operational Considerations:**
- **Lead Time:** Vendor lead times can be long - confirm early in the procurement process
- **Support:** Check vendor support and service options
- **Integration:** WHOI drivers support Teledyne RDI; check vendor ROS support
- **Spares:** Budget for protective caging and spares as needed

#### Inertial Navigation Systems (INS) / IMU

High-accuracy orientation and acceleration measurement.

**Manufacturers:**
- SBG Systems (Ellipse series)
- VectorNav
- Xsens
- LORD MicroStrain
- Advanced Navigation
- EXAIL (PHINS INS range)
- Sparton (M.2)
- Kearfott (high-end)

**Integration:**
- Often combined with DVL for dead-reckoning
- GPS integration for surface vehicles
- AHRS (Attitude and Heading Reference System) capabilities

#### INS/IMU Selection Guide

**Comparison Matrix:**

| Vendor | Model | Type | ROS Support | Spec Sheet |
|--------|-------|------|-----------|------------|
| **SBG Systems** | [Ellipse-N](https://www.sbg-systems.com/ins/ellipse-n/) | INS | ✓ [Official ROS 2](https://github.com/SBG-Systems/sbg_ros2_driver) | SBG Systems product page |
| **SBG Systems** | [Ellipse-D](https://www.sbg-systems.com/ins/ellipse-d/) | INS | ✓ Official ROS 2 | SBG Systems product page |
| **VectorNav** | [VN-100](https://www.vectornav.com/products/vn-100) | IMU/AHRS | Community | VectorNav product page |
| **VectorNav** | [VN-200](https://www.vectornav.com/products/vn-200) | INS | Community | VectorNav product page |
| **Xsens** | [MTi-3](https://www.xsens.com/sensor-modules/xsens-mti-3-ahrs) | AHRS | Community | Xsens product page |
| **LORD MicroStrain** | [3DM-GX5-45](https://www.microstrain.com/inertial/3dm-gx5-45) | INS | Community | LORD MicroStrain product page |
| **Advanced Navigation** | [Spatial FOG Dual](https://www.advancednavigation.com/inertial-navigation-systems/fog-gnss-ins/spatial-fog-dual/) | INS | Limited | Advanced Navigation product page |
| **EXAIL** | [PHINS INS range](https://www.exail.com/product-range/inertial-navigation-for-subsea-operations) | INS | Limited | EXAIL product range |
| **Sparton** | [M.2](https://www.spartonnavex.com/inertial-sensors-ahrs-m2) | AHRS/IMU | Limited | Sparton NavEx product page |
| **Kearfott** | [Inertial Measurement Units](https://www.kearfott.com/products/guidance-navigation/inertial-measurement-units/) | High-grade INS | Custom | Kearfott product page |

**Decision Criteria:**

1. **Budget Student/Hobby Project:**
   - IMU/AHRS-only units can be sufficient for basic attitude
   - **Limitation:** Magnetometer-based heading degrades near metal and high-current systems

2. **Research AUV (Pure Underwater):**
   - Prioritize pressure-rated models with strong integration docs
   - Pair with DVL for dead-reckoning accuracy

3. **ASV (Surface Vehicle):**
   - GNSS integration is essential for surface positioning

4. **ROV (Tethered):**
   - IMU-only configurations may be sufficient
   - **Consideration:** Heading can be provided from topside systems if required

5. **High-Accuracy Survey (Precision Required):**
   - Dual-antenna GNSS + high-grade INS for precise heading

**Critical Specifications to Consider:**

- **Gyro Bias Stability:** Lower is better (affects long-term drift); compare vendor specs
- **Depth Rating:** Essential for UUV applications
- **GNSS Integration:** Dual-antenna GNSS improves heading accuracy
- **Magnetic Interference:** Magnetometer-based heading unreliable near thrusters/motors

**ROS Integration Notes:**
- **SBG Systems:** Official [sbg_ros2_driver](https://github.com/SBG-Systems/sbg_ros2_driver) (check repo activity)
- **Others:** Community drivers vary in quality and support
- **Output:** Standard `sensor_msgs/Imu` for most packages

### Acoustic Sensors

#### Imaging Sonar

Provides acoustic "images" of underwater environments.

**Types:**
- Mechanically scanned imaging sonar (Ping360, Tritech Micron)
- Multi-beam imaging sonar (BlueView, Oculus)
- Forward-Looking Sonar (FLS)

**Use Cases:**
- Obstacle avoidance
- Target detection and classification
- Navigation in turbid water
- Underwater inspection

#### Multibeam Sonar

High-resolution bathymetric mapping.

**Applications:**
- Seafloor mapping
- Hydrographic surveys
- Habitat assessment

#### Side-Scan Sonar

Creates sonar images of large seabed areas.

**Applications:**
- Search and survey
- Pipelines and cables
- Archaeological surveys

### Environmental Sensors

#### CTD (Conductivity, Temperature, Depth)

Fundamental oceanographic sensor.

**Measurements:**
- Salinity (via conductivity)
- Temperature
- Pressure/Depth

**Use Cases:**
- Water column profiling
- Ocean monitoring
- Model validation

#### Pressure Sensors

Depth measurement for underwater vehicles.

**Types:**
- Absolute pressure (depth)
- Differential pressure (for flow measurement)
- High-accuracy options for deep missions

### Vision Systems

#### Underwater Cameras

**Challenges:**
- Light attenuation
- Color distortion
- Turbidity
- Biofouling

**Solutions:**
- Low-light cameras
- Strobes/LED lighting
- Stereo camera systems
- Image processing for color correction

#### Topside Cameras

Surface vehicles use standard computer vision cameras plus:
- Thermal cameras (for night operation, SAR)
- Long-range optical cameras
- Pan-tilt-zoom (PTZ) systems

### Positioning Systems

#### Acoustic Positioning

**USBL (Ultra-Short Baseline):**
- Surface transponder tracks underwater vehicle
- Range and bearing measurement
- Accuracy depends on baseline geometry, environment, and system calibration

**LBL (Long Baseline):**
- Network of seafloor transponders
- High accuracy positioning
- Requires deployment and calibration

**SBL (Short Baseline):**
- Array of transducers on surface platform
- Medium accuracy

#### Surface Positioning

- **GNSS/GPS:** Standard for surface vehicles
- **Differential GPS (DGPS):** Enhanced accuracy for surveying
- **RTK GPS:** High-precision positioning (see receiver specs)

## Propulsion & Actuation

### Thrusters

**Types:**
- Brushed DC thrusters (affordable, shorter lifespan)
- Brushless thrusters (expensive, longer lifespan, higher efficiency)
- Azimuth thrusters (vectorable)

**Manufacturers:**
- Blue Robotics (T100, T200, T500)
- Tecnadyne (professional-grade)
- SeaBotix (professional-grade)
- VideoRay (ROV-specific)

**Considerations:**
- Thrust-to-weight ratio
- Power consumption
- Depth rating
- Maintenance requirements

#### Thruster Selection Guide

**Comparison Matrix:**

| Manufacturer | Model | Type | Spec Sheet |
|--------------|-------|------|------------|
| **Blue Robotics** | [T100](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t100-thruster-core/) | Brushed | Blue Robotics product page |
| **Blue Robotics** | [T200](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) | Brushed | Blue Robotics product page |
| **Blue Robotics** | [T500](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t500-thruster/) | Brushed | Blue Robotics product page |
| **SeaBotix** | BTD150 | Brushless | Contact vendor |
| **SeaBotix** | BTD300 | Brushless | Contact vendor |
| **Tecnadyne** | Model 150D | Brushless | Contact vendor |
| **Tecnadyne** | Model 310 | Brushless | Contact vendor |
| **VideoRay** | M3 | Brushless | Contact vendor |

**Decision Criteria:**

1. **Budget Project / Student ROV:**
   - Consider entry-level brushed thrusters and plan for spares

2. **Research AUV (Medium Budget):**
   - Balance thrust margin, integration effort, and spares strategy

3. **Professional ROV (Reliability Critical):**
   - Prefer brushless thrusters with vendor support and documented maintenance intervals

4. **Deep Ocean:**
   - Confirm depth rating and pressure tolerance with the vendor

5. **Long-Duration AUV:**
   - Minimize continuous thruster use; consider control surfaces or glider designs

**Thruster Count by Vehicle Type:**

| Vehicle Type | Typical Configuration | Notes |
|--------------|----------------------|-------|
| Torpedo AUV | Single stern thruster | Surge-focused designs |
| Simple ROV | Differential drive | Multiple thrusters |
| BlueROV2 | Vectored | Full-DOF capable configuration |
| Heavy ROV | Vectored + verticals | Full-DOF with redundancy |
| ASV | Twin differential | Surge/yaw focused |

**Design Trade-Offs:**

**Option A: More Low-Cost Thrusters**
- Higher redundancy
- Easier to swap spares
- Lower initial cost

**Option B: Fewer High-End Thrusters**
- Lower maintenance overhead
- Higher reliability
- Vendor-supported service intervals

**Recommendation by Use Case:**
- **Education/Research (intermittent use):** Entry-level thrusters + spares
- **Commercial/Survey (continuous use):** Brushless, vendor-supported thrusters
- **Deep Ocean:** Depth-rated professional thrusters
- **DIY/Hobby:** Entry-level thrusters

**Integration Considerations:**
- **ESC (Electronic Speed Controller):** Some vendors include ESCs, others require separate procurement
- **Control Interface:** PWM (simple) vs CAN bus (advanced)
- **ROS Integration:** Blue Robotics BasicESC has community ROS support
- **Mounting:** Consider 3D-printed mounts for Blue Robotics, professional brackets for others

### Control Surfaces

For torpedo-shaped AUVs and gliders:
- Rudders
- Elevators
- Ailerons (for some designs)

### Buoyancy Control

For gliders and some AUVs:
- Piston-based buoyancy engines
- Ballast systems

## Power Systems

### Batteries

**Common Chemistries:**
- Lithium-ion: High energy density, common for most vehicles
- Lithium polymer: Flexible form factor
- Alkaline: Simple, safe, lower performance

**Underwater Considerations:**
- Pressure-compensated housings
- Thermal management
- Battery Management Systems (BMS)

### Energy Harvesting

- Solar panels (for surface vehicles)
- Wave energy (experimental)
- Thermal gradients (for gliders)

## Waterproof Enclosures

Essential for protecting electronics underwater.

**Types:**
- Acrylic tubes (transparent, good for cameras)
- Aluminum enclosures (strong, opaque)
- Subsea connectors (SubConn, Blue Robotics penetrators)
- Pressure compensation systems

**Design Considerations:**
- Depth rating
- Corrosion resistance
- Weight (buoyancy management)
- Accessibility for maintenance

## Tether Systems

For ROVs and some test configurations:
- Neutrally buoyant tethers
- Power and communication cables
- Tether management systems
- Fiber optic for high-bandwidth communication

## Manipulators

Underwater robotic arms for intervention tasks.

**Specifications:**
- Degrees of freedom (varies by model)
- Reach
- Payload capacity
- End effector options (grippers, cutters, sensors)

**Manufacturers:**
- Schilling Robotics
- Reach Robotics
- Blueprint Lab (open-source designs)

## Budget Planning

Budget and schedule vary widely by vehicle class, payload, and operating depth. Use vendor quotes and recent project bills of materials for any planning numbers, and confirm lead times with suppliers before committing to a mission schedule.

---

## Component Suppliers

### Marine Electronics

- **Blue Robotics** - UUV components and accessories
- **Teledyne Marine** - Marine sensors and instrumentation
- **Sonardyne** - Acoustic positioning and navigation systems
- **Nortek** - ADCPs and DVLs

### Subsea Connectors

- **MacArtney (SubConn)** - Subsea connectors
- **Seacon** - Professional connectors, wide variety
- **Blue Robotics** - Penetrators (see vendor specs)

### Waterproof Housings

- **Blue Robotics** - Acrylic tubes, aluminum end caps (see vendor specs)
- **Sexton Corporation** - Custom professional housings
- **Custom fabrication** - Local machine shops for aluminum housings

### Where to Save Money

1. **Buy used DVLs** - Check vendor and reseller listings for availability
2. **3D print non-critical parts** - Mounts, fairings, cable management
3. **Community resources** - Use proven designs and reference builds
4. **Simulation first** - Test extensively in Gazebo before hardware
5. **Incremental integration** - Test each sensor individually before full system

### Where NOT to Compromise

1. **Pressure housings** - Leaks can destroy electronics
2. **Connectors** - Connectors are a common failure point
3. **IMU quality** - IMU quality affects navigation accuracy
4. **Battery safety** - Proper BMS prevents fires/damage
5. **Professional help** - Budget for engineering review on critical systems

---

## Sources

**DVL Specifications:**
- [Nortek DVL1000 Datasheet](https://www.nortekgroup.com/export/pdf/DVL1000%20-%204000%20m.pdf)
- [Nortek DVL500 Product Page](https://www.nortekgroup.com/products/dvl500-300-m)
- [Teledyne Marine Tasman DVL](https://www.teledynemarine.com/en-us/products/SiteAssets/RD%20Instruments/Tasman_DVL.pdf)
- [Water Linked DVL A50](https://waterlinked.com/dvl/)

**Thruster Specifications:**
- [Blue Robotics T100 Thruster](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t100-thruster-core/)
- [Blue Robotics T200 Thruster](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/)
- [Blue Robotics T500 Thruster](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t500-thruster/)

**INS/IMU Specifications:**
- [SBG Systems Ellipse-N](https://www.sbg-systems.com/ins/ellipse-n/)
- [SBG Systems Ellipse-D](https://www.sbg-systems.com/ins/ellipse-d/)
- [SBG ROS 2 Driver](https://github.com/SBG-Systems/sbg_ros2_driver)
- [VectorNav VN-100](https://www.vectornav.com/products/vn-100)
- [VectorNav VN-200](https://www.vectornav.com/products/vn-200)
- [Xsens MTi-3](https://www.xsens.com/sensor-modules/xsens-mti-3-ahrs)
- [LORD MicroStrain 3DM-GX5-45](https://www.microstrain.com/inertial/3dm-gx5-45)
- [Advanced Navigation Spatial FOG Dual](https://www.advancednavigation.com/inertial-navigation-systems/fog-gnss-ins/spatial-fog-dual/)
- [EXAIL PHINS INS range](https://www.exail.com/product-range/inertial-navigation-for-subsea-operations)
- [Sparton M.2](https://www.spartonnavex.com/inertial-sensors-ahrs-m2)
- [Kearfott Inertial Measurement Units](https://www.kearfott.com/products/guidance-navigation/inertial-measurement-units/)

**Note:** Prices and specifications subject to change. Contact vendors directly for current pricing and detailed specifications. Lead times can vary significantly based on market conditions.

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
