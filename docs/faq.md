---
title: FAQ
description: Answers to common questions about getting started in marine robotics with ROS - skills, costs, software versions, and the community.
---

# Frequently asked questions

Click a question to open the answer. Cannot find yours? Ask on [ROS Discourse](https://discourse.openrobotics.org/c/community-groups/marine-robotics/36).

## Getting started

??? question "I am brand new. Where do I start?"

    Read [Getting started](getting_started.md). The fastest hands-on path is: install Ubuntu 24.04 and ROS 2 Jazzy, do the ROS 2 beginner tutorials, then [drive a boat in VRX](education_and_tutorials.md#tutorial-1-drive-a-boat-in-vrx).

??? question "Do I need to know programming?"

    Basic programming helps, and you can learn the rest as you go. **Python** is enough for most beginner work in ROS 2. **C++** is common for drivers and performance-critical code. If you have never programmed, a free introductory Python course is a good first step.

??? question "How much math do I need?"

    To start: algebra and trigonometry (angles and headings). Later: vectors and linear algebra (frames and transforms), and some calculus and control theory (controllers and estimators). Simulators and libraries do the heavy math, so you can learn it when you need it.

??? question "Which operating system?"

    **Ubuntu 24.04** for ROS 2 Jazzy, natively if possible. On **Windows**, use WSL 2 with Ubuntu 24.04. On **macOS**, use a Linux virtual machine or a remote Linux computer.

??? question "How long does it take to learn?"

    It depends on your background and goals. Many people can drive a simulated vehicle within a week, and write useful nodes within a few months of regular practice.

## Software

??? question "ROS 1 or ROS 2?"

    **ROS 2.** ROS 1 Noetic, the last ROS 1 release, reached end of life on May 31, 2025. Some older marine packages are ROS 1 only; use them for reference, or port them. For new projects, use ROS 2 Jazzy (LTS until May 2029) or Lyrical (LTS until May 2031). See the [platform baseline](index.md#current-platform-baseline-october-2026).

??? question "Which simulator should I use?"

    For surface vehicles, VRX. For a BlueROV2, Gazebo Harmonic with ArduSub (Orca5 or bluerov2_ignition). For general underwater work with sonar and DVL, POSIM or Stonefish. For realistic images, HoloOcean or OceanSim. For ready-to-launch package application, WWW-POSIM. See the [simulator selection guide](simulation.md#choose-a-simulator).

??? question "Is there a standard set of maritime ROS messages?"

    Use standard ROS messages where they fit, and **marine_msgs** (marine_acoustic_msgs and marine_sensor_msgs) for DVL, sonar, and radar data. It is released for ROS 2 from Humble to Lyrical. See [Message definitions](drivers.md#message-definitions).

??? question "My vehicle turns the wrong way or the EKF diverges. Why?"

    Most often a frame or sign error: NED data published as ENU, compass heading used as ROS yaw, or depth published as positive z. Read [Frames and conventions](conventions.md), then the [EKF known issues](navigation.md#known-issues).

## Physics questions

??? question "Why can't I use GPS underwater?"

    Water absorbs radio signals within centimeters to meters, so satellite signals do not reach a submerged vehicle. Underwater vehicles use dead reckoning (DVL and INS), acoustic positioning (USBL, LBL), and GNSS fixes when they surface. See [Localization](navigation.md).

??? question "What is the difference between an AUV and an ROV?"

    An **ROV** is tethered and piloted from the surface. The tether gives power and high-bandwidth video, so missions last as long as the support ship stays, but the tether limits range and adds drag. An **AUV** has no tether: it carries batteries and runs its mission autonomously, with only slow acoustic communication while submerged. Hybrid vehicles can do both. See [Vehicle types](vehicle_types.md).

??? question "Why is underwater communication so slow?"

    Sound is the only signal that travels far underwater, and the usable acoustic band is narrow. Acoustic modems carry tens of bit/s to tens of kbit/s, with seconds of latency, and suffer from echoes (multipath) and refraction. Vehicles must be autonomous and send only short status messages. See [Communication](communication.md).

## Hardware and cost

??? question "Do I need to buy hardware to start?"

    No. Learn ROS, control, and estimation in simulation first. VRX, Gazebo, and Stonefish are free.

??? question "What is the cheapest way to build something real?"

    Join a team that shares equipment. If you build alone, start with a small tethered ROV (cheaper and safer to test than an AUV), test in a pool, buy used parts where safe, and 3D-print non-critical parts. Do not save on seals, connectors, or battery safety.

??? question "Where do I buy components?"

    Blue Robotics sells thrusters, housings, and electronics for small vehicles. Professional sensors come from vendors such as Teledyne Marine, Nortek, Water Linked, SBG Systems, and Sonardyne. See [Sensors and components](hardware.md), and check the ROS 2 driver status in [Drivers](drivers.md#ros-2-driver-status-by-vendor) before you buy.

??? question "I don't live near the ocean. Can I still do this?"

    Yes. Simulation needs no water, and pools, lakes, and rivers are good test sites. Many teams test locally and travel for sea trials.

## Education and careers

??? question "I'm in high school. Can I take part?"

    Yes. The MATE ROV Competition has classes for school students, and all the software tools are free. See [Events](events.md).

??? question "What background do people in this field have?"

    Robotics, mechanical, electrical, and ocean engineering, computer science, physics, and marine science, and many people who taught themselves. Projects and demonstrated skills matter more than the degree title.

??? question "Where do people work?"

    Ocean research institutions (for example WHOI and MBARI), universities, offshore energy and wind companies, survey companies, defense, aquaculture, environmental monitoring, and robotics startups. Roles include robotics and software engineer, controls engineer, field engineer, ROV pilot and technician, and researcher.

## Community

??? question "Where can I get help?"

    - [ROS Discourse, Marine Robotics](https://discourse.openrobotics.org/c/community-groups/marine-robotics/36) for detailed questions and announcements
    - [Zulip Marine Robotics channel](https://openrobotics.zulipchat.com/#narrow/channel/526058-Marine-Robotics) for quick questions
    - [Robotics Stack Exchange](https://robotics.stackexchange.com/) for general ROS questions (tag `ros2`)

    Include what you tried, the exact error, and your ROS and Ubuntu versions.

??? question "How can I contribute to this site?"

    Fix or add content with the :material-pencil: button on any page, add packages to the catalog, or share field experience. See [Contributing](contributing.md).
