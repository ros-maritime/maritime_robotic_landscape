# Expert Quick Reference

**For experienced roboticists and marine engineers.** Fast access to technical details, package comparisons, and operational considerations without beginner explanations.

---

## Package Quick Lookup

### Autonomy Stacks

| Stack | ROS | Vehicles | Status | Link |
|-------|-----|----------|--------|------|
| **Blue** | ROS 2 | UUVs | See repo | [GitHub](https://github.com/Robotic-Decision-Making-Lab/blue) |
| **MVP** | ROS/ROS 2 | USV/UUV | See repo | [GitHub](https://github.com/uri-ocean-robotics/mvp_readme) |
| **Project11** | ROS 2 | ASV | See repo | [GitHub](https://github.com/CCOMJHC/project11) |
| **Orca4** | ROS 2 | BlueROV2 | See repo | [GitHub](https://github.com/clydemcqueen/orca4) |

[Full comparison →](autonomy_stacks.md)

### Thrust Allocation

| Package | Method | Handles Saturation | Azimuth Support |
|---------|--------|-------------------|-----------------|
| **MVP-Control** | QP optimization | ✓ Yes | ✓ Yes |
| **thruster_manager** | Pseudo-inverse | ✗ No | ✗ No |
| **thruster_allocation_matrix** | Pseudo-inverse | ✗ No | ✗ No |

**When to use QP:** Non-square allocation matrices, hard constraints (saturation, dead zones)
**When to use pseudo-inverse:** Square matrices, known unsaturated operation, speed critical

[Details →](software.md#thrust-allocation-and-control)

### Sensor Drivers

**Navigation:**
- **DVL:** whoi_ds (Teledyne RDI), ~~waterlinked_dvl~~ (archived repo; last push 2024-02-26)
- **INS/IMU:** sbg_ros2_driver (SBG Systems), standard sensor_msgs compatible
- **USBL:** Vendor-specific, integrate via marine_msgs

**Acoustic:**
- **Communication:** ros_acomms ⭐ (WHOI)
- **Sonar:** Vendor-specific (Blue Robotics ping360_sonar for Ping360)

[Full driver list →](drivers.md)

### Simulators

Refer to each simulator's documentation for engine version, ROS compatibility, hardware requirements, and supported features.

- **VRX:** https://github.com/osrf/vrx
- **DAVE:** https://github.com/Field-Robotics-Lab/dave
- **HoloOcean:** https://holoocean.readthedocs.io/
- **UNav-Sim:** https://github.com/open-airlab/UNav-Sim
- **Stonefish:** https://stonefish.readthedocs.io/en/latest/

⚠️ **UUV Simulator archived** (see [uuvsimulator/uuv_simulator](https://github.com/uuvsimulator/uuv_simulator))
⚠️ **DAVE on ROS 1** - ROS Noetic EOL is 2025-05-31 (see [ROS notice](https://www.ros.org/blog/noetic-eol/))

[Simulator comparison →](simulation.md)

---

## Common Integration Patterns

### Standard AUV Stack

```
DVL + INS + Depth Sensor
         ↓
   robot_localization (EKF)
         ↓
   Navigation/Planning
         ↓
   Thrust Allocation
         ↓
      Thrusters
```

**Typical message flow:**
- DVL → `geometry_msgs/TwistWithCovarianceStamped` (velocity)
- INS → `sensor_msgs/Imu` (orientation, angular velocity)
- Depth → `sensor_msgs/FluidPressure` or custom
- EKF → `nav_msgs/Odometry` (fused state estimate)

[Full integration examples →](integration_examples.md)

### Reference tf Trees

**Torpedo AUV:**
```
map → odom → base_link → dvl_link
                       → imu_link
                       → sonar_link
```

**Hovering ROV:**
```
world → odom → base_link → camera_link
                         → dvl_link
                         → imu_link
                         → thruster_[1-8]_link
```

**Key frames:**
- `map`: World-fixed (global reference)
- `odom`: Continuous but drifting (from dead reckoning)
- `base_link`: Robot body center

---

## Performance Notes

- QP-based allocators trade higher solver latency for constraint handling; pseudo-inverse methods are faster but ignore constraints.
- robot_localization performance depends on sensor rates, hardware, and covariance tuning; test on target platforms.
- Simulator performance varies widely with scene complexity and GPU capability; validate real-time behavior on your hardware.

---

## Known Issues & Limitations

### DVL Integration

**Common problems:**
- ✗ Bottom-lock can fail at higher altitude (vehicle-dependent)
- ✗ Soft sediment reduces accuracy (mud, silt)
- ✗ Acoustic interference with sonar (stagger pings)
- ✗ Cavitation noise from thrusters corrupts measurements
- ✓ **Solution:** Configure DVL ping timing, reduce thruster noise via mounts

**Water-track vs. bottom-track:**
- Bottom-track: Accurate but needs seafloor within range
- Water-track: Works mid-water but assumes no current (poor accuracy)

### Acoustic Communication

**ros_acomms limitations:**
- Bandwidth: Low relative to RF links
- Latency: High relative to RF links
- Dropout: Common in noisy environments
- **Not suitable for:** Real-time control, video streaming, large file transfer
- **Suitable for:** Status updates, waypoint updates, emergency commands

**Environmental factors:**
- Multipath: Shallow water, near structures
- Thermoclines: Bends sound, reduces range
- Biologics: Snapping shrimp interfere
- Surface conditions: Bubbles from waves degrade signal

### Simulation-to-Reality Gap

**VRX:**
- ✗ Wave modeling simplified (not full sea state modeling)
- ✗ Sensor noise models optimistic
- ✓ Good for algorithm development, not hardware tuning

**DAVE:**
- ✗ Hydrodynamics simplified vs. real vehicle
- ✗ Sonar models lack multipath, reverberation
- ✓ Useful for integration testing, not performance validation

**General:**
- Controllers tuned in sim always need retuning on hardware
- Expect additional debugging time for real-world deployment

---

## Vendor Comparison Criteria

Use manufacturer datasheets and documented field experience to compare vendors. Prioritize:

- Depth rating and environmental limits
- Power and interface requirements
- Accuracy and drift specifications
- Support and serviceability
- Lead times and supply stability

---

## Operations Checklist (Quick)

**Pre-deployment:**
- [ ] DVL warm-up (per manufacturer guidance)
- [ ] IMU calibration check
- [ ] Acoustic modem range test (surface comms verify)
- [ ] Thruster function test (all directions)
- [ ] GPS fix acquired (for surface positioning)
- [ ] Depth sensor zero-offset calibration
- [ ] Camera focus check (underwater = different focal length)
- [ ] Emergency abort tested

**Post-deployment:**
- [ ] Data download initiated immediately (don't wait - flash can corrupt)
- [ ] Sensor rinse (freshwater for salt deployments)
- [ ] Battery voltage check (log degradation)
- [ ] O-ring inspection
- [ ] Thruster blockage check (seaweed, fishing line)

[Full operations guide →](operations.md)

---

## Quick Troubleshooting

### "DVL shows no bottom lock"

**Checklist:**
1. Too far from bottom? (check bottom-track range)
2. Soft bottom (mud/silt)? (Weak returns)
3. Thruster cavitation noise? (Mask DVL)
4. Configuration: Check beam angle, frequency

**Quick fix:** Reduce altitude or switch to water-track (less accurate)

### "Acoustic modem not connecting"

**Checklist:**
1. Range test first (before blaming software)
2. Check frequency match (TX and RX same freq)
3. Environmental: Thermocline, bubbles, biologics?
4. Timing: Are both modems listening/talking at right times?

**Quick fix:** Increase TX power, reduce data rate, simplify message

### "EKF diverging"

**Root causes:**
- Sensor timestamp mismatch
- Incorrect covariances (too confident in bad sensor)
- Bad initial state
- TF tree broken

**Quick fix:** Check `rqt_graph`, verify timestamps, reduce sensor weight

### "Thrusters saturating"

**Causes:**
- Overweight vehicle (poor buoyancy trim)
- Current too strong (beyond vehicle capability)
- Allocation matrix poorly conditioned

**Quick fix:** Improve buoyancy, reduce commanded velocities, check allocation matrix condition number

---

## Fast Links for Experts

**Package repositories:**
- [ros-maritime GitHub](https://github.com/ros-maritime)
- [WHOI GitLab](https://git.whoi.edu/)
- [Blue Robotics](https://github.com/bluerobotics)

**Technical references:**
- [robot_localization docs](https://docs.ros.org/en/melodic/api/robot_localization/html/)
- [ros2_control](https://control.ros.org/)
- [Gazebo](https://gazebosim.org/)

**Community:**
- [ROS Discourse - Maritime](https://discourse.ros.org/c/maritime/36)
- [Zulip Chat](https://openrobotics.zulipchat.com/#narrow/channel/526058-Marine-Robotics)

**Key papers:**
- Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control"
- Gallimore et al., "ROS Message Transport over Underwater Acoustic Links with ros_acomms," IEEE/OES AUV 2022

---

## Contributing Expert Knowledge

**We need your operational experience:**

- **Known issues you've hit** - Add to package pages
- **Performance benchmarks** - Real hardware test results
- **Integration examples** - Your working launch files
- **Vendor comparisons** - What worked, what didn't
- **Lessons learned** - Sea trial wisdom

**How to contribute:**
- Quick: Comment on [Discourse](https://discourse.ros.org/c/maritime/36)
- Better: Edit pages via "Edit on GitHub" (pull request)
- Best: Join monthly WG meetings, present your experience

**Your late-night debugging sessions can save someone else's deployment.**

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
