# Frequently Asked Questions

Common questions from beginners about getting started with marine robotics. Can't find your question? Ask on [ROS Discourse](https://discourse.ros.org/c/maritime/36)!

## Getting Started

### I'm brand new. Where do I start?

Start with the [Getting Started Guide](getting_started.md). It walks you through three paths depending on your commitment level, from just watching videos to building your first robot.

The quickest path to hands-on experience:
1. Read [What is Marine Robotics?](getting_started.md#what-is-marine-robotics)
2. Install a simulator
3. Complete [Tutorial 1: VRX](education_and_tutorials.md#tutorial-1-setting-up-vrx-simulation)

### Do I need to know programming?

**Short answer:** Basic programming helps, but you can learn as you go.

**Long answer:**
- **Python** is commonly used in ROS 2 - it's beginner-friendly
- **C++** is used for performance-critical code, but you can start without it
- Many tutorials provide copy-paste code to get you started
- If you've never programmed: Try [Python for Everybody](https://www.py4e.com/) (free course) first

### Do I need to know advanced math?

**No!** Basic algebra is enough to start for many entry-level tasks.

**What you'll encounter:**
- **Basic algebra:** For simple calculations
- **Trigonometry:** For navigation (sin, cos for angles)
- **Linear algebra:** For coordinate transformations (you'll learn this)
- **Calculus:** For advanced control (not needed initially)

The simulators and libraries handle most complex math for you.

### What operating system do I need?

**Best:** Linux (use the version supported by your ROS 2 distribution)
- ROS 2 works best on Linux
- Most documentation assumes Linux
- Free to install

**Windows:** Use WSL 2 (Windows Subsystem for Linux)
- Lets you run Linux inside Windows
- Free and officially supported
- [Installation guide](https://docs.microsoft.com/en-us/windows/wsl/install)

**Mac:** Partial support
- Some simulators work
- ROS 2 can be installed but fewer packages available
- Consider running Linux in a virtual machine

### How long does it take to learn?

Timelines vary widely based on prior experience, project scope, and access to hardware. Most people learn incrementally while working on projects that interest them.

## Hardware and Costs

### Do I need to buy hardware to start?

**No!** Start with free simulators:
- VRX (Virtual RobotX) - Surface vehicle simulation
- DAVE - Underwater simulation
- Gazebo - General robotics simulator

You can learn ROS, navigation, control, and sensors all in simulation without spending money.

### How much does hardware cost?

Hardware costs range from DIY builds to professional systems. Prices depend on configuration, payloads, and depth ratings.

**Recommendation:** Join a university lab or competition team to share costs. Prices vary by region, vendor, and configuration.

### What's the cheapest way to build something real?

**Most affordable path:**

1. **Join a team** - Share equipment, tools, and knowledge
2. **Pool robot in simulation** → **pond/pool testing** → **ocean deployment**
3. **Start tethered (ROV)** - Cheaper than autonomous (AUV)
4. **Buy used components** - Check robotics forums, university surplus sales
5. **3D print custom parts** - Access makerspaces for tools

**Realistic first project:** Tethered camera robot using low-cost parts.

### Where can I buy marine robotics components?

**Popular suppliers (examples):**
- **Blue Robotics** - Thrusters, controllers, ROV components (beginner-friendly)
- **Teledyne** - Professional sensors
- **Amazon/eBay** - Generic electronics, Arduino, Raspberry Pi
- **McMaster-Carr** - Mechanical components, hardware
- **3D printing services** - Custom parts (Shapeways, local makerspaces)

**University students:** Check if your institution has equipment you can use!

## Education and Age

### I'm in high school. Can I participate?

**Absolutely!** Many opportunities for high school students:

**Competitions:**
- MATE ROV Competition (specifically for students)
- SAUVC (Singapore AUV Challenge)
- Local robotics competitions

**Learning:**
- All online tutorials are free
- Many simulators run on modest computers
- Online community is very welcoming

**Funding:** Some competitions provide equipment or have student grants; check individual competition resources.

### I'm not in college. Can I still learn?

**Yes!** This field welcomes:
- **Hobbyists** - Build for fun
- **Career changers** - Transition from other fields
- **Retirees** - Pursue a passion project
- **Self-taught learners** - Online resources are excellent

The ROS community values skills and enthusiasm over formal credentials.

### What degree/major do I need?

**Common backgrounds:**
- Robotics Engineering
- Mechanical Engineering
- Electrical Engineering
- Computer Science
- Ocean Engineering
- Marine Biology (for science payload/applications)

**But:** Many professionals are self-taught or came from unrelated fields. Projects and skills matter more than the degree title.

### Can I get a job in marine robotics?

**Yes!** There are opportunities in both industry and research:

**Job titles:**
- Robotics Engineer
- Ocean Technology Specialist
- AUV/ROV Pilot & Technician
- Controls Engineer
- Marine Robotics Researcher
- Simulation Developer
- Sensor Integration Engineer

**Employers:**
- Ocean research institutions (WHOI, MBARI, etc.)
- Defense contractors
- Offshore energy companies
- Environmental monitoring firms
- Underwater robotics startups
- Universities and research labs

**Salary range:** Varies widely by region, role, and experience. Check local market data for current ranges.

## Technical Questions

### What's the difference between ROS and ROS 2?

**ROS (ROS 1):**
- Original version
- Still used in many existing systems
- ROS 1 Noetic EOL: 2025-05-31 (older ROS 1 distributions are already EOL) — see [ROS notice](https://www.ros.org/blog/noetic-eol/)

**ROS 2:**
- Modern rewrite with better architecture
- Real-time capable
- Better security
- **Recommended for new projects**

**Should I learn ROS 1 or ROS 2?**
Start with ROS 2. Some older packages only work with ROS 1, but ROS 2 is the current focus for new development.

### Why can't I use GPS underwater?

**Physics:** Radio waves (including GPS signals) are absorbed very quickly by water. GPS signals are effectively unusable underwater.

**Solutions for underwater navigation:**
- **DVL (Doppler Velocity Log):** Measures speed relative to seafloor
- **INS/IMU:** Tracks position by measuring acceleration (drifts over time)
- **Acoustic positioning:** USBL or LBL systems use underwater sound beacons
- **Sensor fusion:** Combine all sensors using Kalman filters

### What's the difference between an AUV and an ROV?

**ROV (Remotely Operated Vehicle):**
- Connected by tether (cable) to surface
- Human operator controls it in real-time
- Extended mission duration (powered via cable)
- Can transmit live video
- Good for: Inspection, deep sea work, tasks needing human judgment

**AUV (Autonomous Underwater Vehicle):**
- No tether; operates independently
- Follows pre-programmed mission or makes own decisions
- Limited by battery life (mission duration depends on vehicle design)
- Limited communication while submerged
- Good for: Mapping, surveys, long-distance missions

**Hybrid vehicles** exist that can operate in both modes!

### Why is underwater communication so slow?

**Acoustic communication limitations:**
- **Low bandwidth:** Much lower throughput than WiFi or RF links
- **High latency:** Seconds of delay are common
- **Multipath:** Sound bounces off surface, seafloor, creating echoes
- **Doppler effects:** Moving vehicles shift frequencies

**Physics:** Sound is the only thing that travels well underwater. Light and radio waves don't. We're stuck with the limitations of acoustic communication.

**Result:** Can't stream video or send large files. Need to compress data heavily.

## Community and Support

### I'm stuck. Where can I get help?

**For technical questions:**

1. **ROS Discourse - Maritime Category** (best for detailed questions)
   - [discourse.ros.org/c/maritime](https://discourse.ros.org/c/maritime/36)
   - Searchable archive
   - Experts monitor regularly

2. **Matrix Chat** (for quick questions)
   - [#ros-maritime](https://matrix.to/#/#ros-maritime:matrix.org)
   - Real-time chat
   - Friendly community

3. **Stack Overflow** (for programming questions)
   - Tag with `ros2` and `marine-robotics`

**Before asking:**
- Check the [Glossary](glossary.md)
- Search previous discussions
- Read relevant [tutorials](education_and_tutorials.md)
- Include error messages and what you've tried

### Are there mentorship programs?

**Formal programs:**
- Some university labs offer mentorship
- Competition teams often pair beginners with experienced members
- Google Summer of Code (for ROS projects)

**Informal mentorship:**
- Monthly ROS Maritime Working Group meetings - watch and learn
- Community members often help on Discourse and Matrix
- Many people share contact info if you're working on interesting projects

### Can I contribute to this landscape/community?

**Absolutely! Contributions welcome:**

**Easy contributions:**
- Fix typos or unclear wording (click "Edit on GitHub")
- Suggest missing resources
- Share your experience as a beginner (helps us improve!)

**Medium contributions:**
- Add tutorials or guides
- Write about your projects
- Review and test existing tutorials

**Advanced contributions:**
- Develop new packages
- Improve simulation environments
- Contribute to core ROS packages

See [Contributing](index.md#contributing) for details.

## Competitions

### Should I join a competition as a beginner?

**Pros:**
- Structured learning path
- Team environment (shared knowledge and costs)
- Access to equipment
- Motivation and deadlines
- Resume-building experience

**Cons:**
- Time commitment (especially near competition)
- Pressure can be stressful
- May need to learn quickly

**Recommendation:** Competitions can be a strong learning path. Check division requirements and time commitments before committing.

### Which competition is best for beginners?

**Common entry points:**

1. **VRX (Virtual RobotX)** - Simulation-based, no hardware needed
2. **MATE ROV** - Student-focused, multiple divisions
3. **SAUVC** - Pool-based, good for students

**More advanced:**
- RobotX (requires team and funding)
- MBZIRC (professional level)

**Recommendation:** Start with VRX (virtual) to learn, then explore hardware-based competitions if you want physical build experience.

### Do competitions provide equipment?

**Varies by competition:**

**VRX:** No physical equipment needed - all simulation
**MATE ROV:** Teams build their own robots (check rules and cost expectations)
**RobotX:** Competition provides a standardized platform (check current rules)
**University competitions:** Often lab equipment available to team members

**Check each competition's rules** for specifics.

## Practical Concerns

### I don't live near the ocean. Can I still do marine robotics?

**Yes!** Many opportunities without ocean access:

- **Simulation:** All learning can happen on your computer
- **Pools:** Test in swimming pools or university pool facilities
- **Lakes and ponds:** Good for surface vehicles and shallow tests
- **Virtual competitions:** VRX requires no water at all
- **Eventually:** Travel to ocean for final tests (field trips, team deployments)

Many people start in simulation and build toward field work over time.

### Is marine robotics only for ocean research?

**No!** Applications beyond ocean science:

- **Infrastructure inspection:** Bridges, dams, water towers, ship hulls
- **Environmental monitoring:** Lakes, rivers, reservoirs
- **Aquaculture:** Fish farming automation
- **Search and rescue:** Finding drowning victims, crashed aircraft
- **Underwater construction:** Assistance with building and repairs
- **Military/Defense:** Mine detection, harbor security
- **Archaeology:** Shipwreck discovery and documentation
- **Energy:** Offshore wind, oil & gas inspection

### What if I can't afford expensive sensors?

**Work-arounds:**

1. **Simulation:** Test algorithms with virtual sensors (free)
2. **Start basic:** Use readily available sensors and build up over time
3. **Share equipment:** University labs, makerspaces, team resources
4. **Open-source alternatives:** Community DIY sensor projects
5. **Used equipment:** Check forums, eBay, university surplus
6. **Grant funding:** Check local grants or sponsorships for student teams

**Remember:** Professionals often start with basic sensors and add capability incrementally.

## Still Have Questions?

- **Quick terms:** Check the [Glossary](glossary.md)
- **Getting started:** See the [Getting Started Guide](getting_started.md)
- **Technical help:** Ask on [ROS Discourse](https://discourse.ros.org/c/maritime/36)
- **Real-time chat:** Join [Matrix #ros-maritime](https://matrix.to/#/#ros-maritime:matrix.org)

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
