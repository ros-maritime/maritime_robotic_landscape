# Operations and Deployment

Practical guidance for deploying marine robots based on real-world experience. This page covers pre-deployment checklists, power budgets, failure modes, environmental limits, and operational procedures.

!!! warning "Safety First"

    Marine robotics operations involve expensive equipment, challenging environments, and potential hazards. Always:

    - Have emergency recovery procedures
    - Never deploy alone; ensure adequate staffing
    - Check weather and sea state forecasts
    - Brief all personnel on safety protocols
    - Have communication backup plan

---

## Pre-Deployment Checklist

### Day Before Deployment

**Vehicle Preparation:**
- [ ] Battery fully charged and voltage verified
- [ ] All O-rings inspected (no nicks, cuts, or compression set)
- [ ] O-rings lubricated with appropriate grease (silicone or Krytox)
- [ ] Pressure housing leak tested (vacuum test if possible)
- [ ] All sensors functional (bench test in air)
- [ ] Software updated and tested in simulation
- [ ] Mission plan loaded and verified
- [ ] Data storage cleared (ensure sufficient space)
- [ ] Backup battery charged (for recovery)

**Support Equipment:**
- [ ] Launch/recovery equipment tested
- [ ] Spare parts packed (O-rings, connectors, tools)
- [ ] Communication equipment charged
- [ ] Shore computer ready with monitoring software
- [ ] Emergency contact numbers programmed
- [ ] Weather forecast reviewed

**Documentation:**
- [ ] Mission plan documented
- [ ] Risk assessment completed
- [ ] Permits/permissions verified (if required)
- [ ] Insurance current
- [ ] Emergency procedures reviewed with team

### Shortly Before Splash

**System Power-Up Sequence:**

1. **DVL warm-up** (per manufacturer guidance)
   - Power on DVL first
   - Allow transducers to stabilize temperature
   - Verify bottom-lock acquisition (on deck/dock if possible)

2. **IMU calibration**
   - Power on IMU
   - Allow gyro bias stabilization per manufacturer guidance
   - Verify orientation output makes sense

3. **GPS fix acquisition**
   - Power on GPS
   - Wait for a reliable fix (cold starts can take longer)
   - Log initial position as deployment reference

4. **Depth sensor zero-offset**
   - Power on with sensor at surface (in air or just submerged)
   - Record zero-offset (atmospheric pressure)
   - Verify the reading aligns with local conditions

5. **Acoustic modem range test**
   - Power on modem
   - Perform surface range test with base station
   - Verify bidirectional communication
   - Test emergency abort command

6. **Thruster function test**
   - **Critical: Keep vehicle restrained**
   - Command each thruster individually
   - Verify all directions (forward/reverse)
   - Check for abnormal noise or vibration
   - Verify thrust allocation produces expected combined motion

7. **Camera/lights check**
   - Power on cameras
   - Verify image quality and focus
   - Check lights (may need submersion to verify thermal performance)

**Final Checks:**
- [ ] All hatches properly sealed
- [ ] No loose cables or items that could snag
- [ ] Abort/recovery line attached (if applicable)
- [ ] Team briefed on deployment and recovery procedures
- [ ] Visual inspection complete (no obvious damage)

### During Deployment

**Launch phase:**
- Monitor system status in real-time
- Verify sensor data immediately after submersion
- Abort if any critical sensor failures
- Log deployment time and GPS position

**Mission phase:**
- Monitor periodically (acoustic status updates if available)
- Log any anomalies or environmental changes
- Be ready for emergency recovery
- Track estimated mission completion time

**Recovery phase:**
- Be ready at estimated surface time
- Have visual line-of-sight before approach
- Secure vehicle promptly (don't let it drift)
- Power down in reverse order of power-up

### Post-Deployment

**Immediate (post-recovery):**
- [ ] Download data IMMEDIATELY (don't wait - corruption risk increases with time)
- [ ] Backup data to multiple locations
- [ ] Rinse vehicle with fresh water (if salt water deployment)
- [ ] Remove batteries and dry compartments
- [ ] Inspect all seals and hatches
- [ ] Log any damage or issues

**Same day:**
- [ ] Review data quality and mission success
- [ ] Document lessons learned
- [ ] Note any anomalies or failures
- [ ] Update maintenance log
- [ ] Charge batteries for next deployment

**Within week:**
- [ ] Full inspection of all O-rings
- [ ] Sensor calibration check
- [ ] Review and archive mission data
- [ ] Update procedures based on lessons learned

---

## Power Budgets

### Calculating Mission Duration

**Basic formula:**
```
Mission Duration = Battery Capacity (Wh) / Average Power Draw (W) × Efficiency Factor
```

**Efficiency factor:** Use a conservative factor that accounts for discharge curves, temperature, and aging. Check your battery datasheet.

### Example Power Budget - Hovering AUV

**Note:** This is a template structure. Fill with values from your component datasheets and measurements.

| Component | Power (W) | Duty Cycle | Avg Power (W) | Notes |
|-----------|-----------|------------|---------------|-------|
| **Onboard Computer** | TBD | TBD | TBD | Datasheet + measured load |
| **DVL** | TBD | TBD | TBD | Datasheet |
| **IMU** | TBD | TBD | TBD | Datasheet |
| **Depth Sensor** | TBD | TBD | TBD | Datasheet |
| **Sonar** | TBD | TBD | TBD | Datasheet |
| **Cameras/Lights** | TBD | TBD | TBD | Datasheet |
| **Thrusters** | TBD | TBD | TBD | Measured under mission profile |
| **Acoustic Modem** | TBD | TBD | TBD | Datasheet |
| **Miscellaneous** | TBD | TBD | TBD | Converters, comms, auxiliaries |
| **Total** | | | **TBD** | |

**Margin:** Reserve capacity for contingencies and recovery. Document your policy in the mission plan.

### Power Optimization Strategies

**High-power consumers:**
1. **Thrusters** - Often the largest consumer during transit
   - Minimize aggressive maneuvering
   - Use efficient paths
   - Consider buoyancy gliding if applicable

2. **Lights** - Necessary for imaging but power-hungry
   - Only enable when imaging
   - Use minimum brightness needed
   - Consider strobes vs continuous

3. **Computers** - Always-on baseload
   - Use efficient processors where feasible
   - Disable unused cores/features
   - Optimize software (avoid busy-wait loops)

**Low-power mode:**
- Disable cameras/lights during transit
- Reduce sonar ping rate
- Lower computer CPU frequency
- Can extend mission duration depending on your hardware

---

## Environmental Operating Limits

### Sea State Limits

**Surface vehicles (ASV/USV):**

Define go/no-go criteria based on vehicle design, operator experience, and local marine forecasts. Document limits in your SOPs and update them based on field experience.

**Limitations to consider:**
- GPS accuracy degrades (antenna motion)
- Acoustic modem performance drops (surface bubbles)
- Solar panels less effective (shadowing from waves)
- Hull slamming can damage equipment

**UUV deployment:**
- Launch/recovery most critical phase
- Launch/recovery often has stricter limits than submerged operation
- Once submerged, UUV largely unaffected by surface conditions
- Deep thermoclines caused by storms can affect acoustics

### Current Limits

**Vehicle capability:**
- Strong currents can prevent waypoint following and increase power consumption
- Define abort criteria based on your vehicle's measured performance

**Impact:**
- Increased power consumption (fighting current)
- Poor station-keeping
- Mission duration reduced
- Path deviations increase

**Mitigation:**
- Plan missions with/across current (not against)
- Increase speed margins in planning
- Use current profiles if available
- Have abort criteria (if drift exceeds X meters)

### Temperature Effects

**Battery performance:**
- Capacity drops in cold conditions
- Very cold environments can cause permanent damage
- Use manufacturer derating curves when available

**Electronics:**
- Temperature ratings vary by component
- Pressure housings can help stabilize temperature
- Sensors may need time to stabilize

**Acoustic propagation:**
- Thermoclines bend sound (can create shadow zones)
- Warm surface, cold deep = sound bends down
- Affects acoustic modem range and USBL accuracy

### Depth Limitations

**Design implications:**
- Verify depth ratings for every pressure-bearing component
- Pressure housing cost increases with depth
- DVL range and acoustic performance can limit operations

---

## Failure Modes and Recovery

### Critical Sensor Failures

#### DVL Failure

**Symptoms:**
- No bottom lock
- Erratic velocity readings
- Beam status errors

**Immediate actions:**
1. Switch to IMU-only dead reckoning (short term)
2. Surface for GPS fix if possible
3. Reduce mission to lower speed (dead reckoning degrades with speed)
4. Abort if position accuracy critical

**Prevention:**
- Pre-mission warm-up
- Check beam status before critical maneuvers
- Plan missions within bottom-lock range

#### IMU Failure

**Symptoms:**
- Orientation divergence
- Attitude solution errors
- High residuals in EKF

**Immediate actions:**
1. If hovering ROV: Surface immediately (no attitude = no control)
2. If torpedo AUV: May continue with degraded performance
3. Check magnetometer (often fails separately from gyros)
4. Attempt recalibration if possible

**Prevention:**
- Pre-flight calibration
- Avoid magnetic interference sources
- Temperature stabilization time

#### GPS Failure (Surface Vehicle)

**Symptoms:**
- No fix or stale fix
- Large position jumps

**Immediate actions:**
1. Continue with dead reckoning (DVL + IMU if available)
2. Check antenna connection
3. Check for obstructions (debris on antenna)
4. Try rebooting GPS receiver

**Recovery:**
- Most failures are cable/connector issues
- Salt buildup can block antenna
- Requires return to shore for repair

#### Acoustic Modem Failure

**Symptoms:**
- No responses to commands
- Corrupted messages
- Complete silence

**Immediate actions:**
1. Vehicle continues autonomous mission (if programmed)
2. Wait for scheduled surface (can't command abort)
3. Monitor expected surface location for recovery

**Prevention:**
- Always program autonomous abort conditions
- Don't rely solely on acoustic for critical commands
- Have time-based or location-based automatic surface

### Mechanical Failures

#### Thruster Failure

**Single thruster:**
- Thrust allocation can often compensate
- Reduced performance but mission may continue
- Monitor power draw on remaining thrusters

**Multiple thrusters:**
- Abort mission
- Activate emergency surface procedure
- Jettison weights if equipped

**Common causes:**
- Seaweed/debris entanglement
- Electrical short
- Bearing failure (noise, vibration)

#### Leak Detection

**Symptoms:**
- Water in pressure housing
- Conductivity sensor alarm
- Unexplained power loss

**Immediate actions:**
1. **Emergency surface immediately**
2. All thrusters full up
3. Drop weights if equipped
4. Activate recovery beacon

**Prevention:**
- Careful O-ring maintenance
- Vacuum test before deployment
- Don't over-torque hatches

### Software/Control Failures

#### Controller Instability

**Symptoms:**
- Oscillation in position or attitude
- Growing amplitude oscillations
- Erratic thruster commands

**Immediate actions:**
1. Switch to backup controller (if available)
2. Reduce controller gains
3. Emergency surface if unsafe

**Common causes:**
- Incorrect tuning for actual vehicle
- Sensor noise not filtered
- Actuator saturation causing integrator windup

#### EKF Divergence

**Symptoms:**
- Position estimate jumps
- High innovation values
- Covariance growing unbounded

**Immediate actions:**
1. Reset EKF if possible
2. Disable faulty sensor input
3. Switch to primary sensor only (e.g., DVL-only navigation)

**Prevention:**
- Proper covariance tuning
- Sensor timeout detection
- Outlier rejection

---

## Data Management

### Mission Data Collection

**Essential logging:**
- All sensor raw data (for post-processing)
- State estimates (position, velocity, attitude)
- Control commands (thruster outputs)
- System status (CPU, power, temperatures)
- Timestamps (synchronized across all sensors)

**Data rates:**
- Sensor rates vary widely by hardware and configuration
- Estimate storage based on your configured rates and payloads

### Post-Mission Processing

**Quality checks:**
1. **Sensor health:** Any dropouts or errors?
2. **Position accuracy:** Compare GPS fixes (if available)
3. **Control performance:** Did vehicle follow commands?
4. **Power consumption:** Match predictions?

**Data products:**
- Processed navigation (final position estimates)
- Georeferenced sensor data (imagery, sonar)
- Mission performance metrics
- Lessons learned documentation

---

## Standard Operating Procedures (SOP) Template

### Mission Planning SOP

1. **Define mission objectives** (survey area, inspection target, etc.)
2. **Plan waypoints** with environmental considerations
3. **Calculate power budget** and mission duration
4. **Check environmental forecast** (sea state, current, temperature)
5. **Identify abort criteria** (sensor failures, position error limits, time limits)
6. **Review emergency procedures** with team
7. **Obtain required permissions** (if applicable)

### Deployment SOP

1. **Pre-deployment checklist** (see above)
2. **Team brief** (roles, communication protocol, abort signals)
3. **Power-up sequence** (DVL first, thrusters last)
4. **Function tests** (all sensors, thrusters, modem)
5. **Launch** (controlled, documented)
6. **Monitor** (periodic status checks)

### Recovery SOP

1. **Estimate surface location** (last known position + drift)
2. **Visual acquisition** (binoculars, spotter)
3. **Secure vehicle** (recovery line/hook)
4. **Power down** (reverse of power-up sequence)
5. **Immediate post-recovery** (data download, rinse, inspect)

---

## Lessons Learned (Community Contributed)

!!! tip "Share Your Experience"

    Have operational wisdom to share? Add to this page via pull request or post on [ROS Discourse](https://discourse.ros.org/c/maritime/36).

**From the community:**

### "Always test acoustics first"
*Contributor: Field roboticist*

"After loading vehicle on boat and transiting to site, first thing we do is acoustic range test before launching vehicle. Saves hours of debugging if modem isn't working. Can't tell you how many times we caught a bad cable or wrong configuration before splashing."

### "GPS cold start takes longer than you think"
*Contributor: ASV operator*

"Budget extra time for GPS to get a solid fix after power-on. Don't rush this. A poor GPS fix at start can corrupt navigation."

### "DVL needs warm-up"
*Contributor: Research engineer*

"DVL transducers can drift during warm-up. We power our DVL early and wait for readings to stabilize before launch."

### "Fresh water rinse saves equipment"
*Contributor: Marine technician*

"After every salt water deployment: thorough fresh water rinse. We've saved thousands in corrosion damage with this simple step. Pay attention to connectors - they corrode first."

### "Document everything"
*Contributor: Ocean engineer*

"Maintain a detailed log book. Date, time, location, sea state, what worked, what didn't. Future you will thank past you when troubleshooting recurring issues."

### "Spare O-rings are mission-critical"
*Contributor: ROV pilot*

"Keep a complete set of spare O-rings at deployment site. Murphy's law: you'll nick an O-ring during assembly. Having spares means mission continues; not having them means mission aborted."

---

## Maintenance Schedule

### After Each Deployment
- Fresh water rinse
- Visual inspection
- Data download and backup
- Battery recharge
- O-ring inspection

### Weekly (Active Use)
- Thruster bearing check
- Connector inspection
- Software updates review
- Battery health check

### Monthly
- Full system test
- O-ring replacement (preventive)
- Sensor calibration verification
- Vacuum leak test

### Annually
- Complete overhaul
- Pressure housing hydro test
- All O-rings replaced
- Battery capacity test
- Professional calibration of critical sensors (DVL, INS)

---

*For deployment-specific guidance, see vehicle manufacturer's documentation. This page provides general best practices from community experience.*

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
