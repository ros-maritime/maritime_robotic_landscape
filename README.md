# Maritime Robotics Landscape

A comprehensive collection of resources for maritime robotics development with ROS.

This landscape covers:
- Surface vehicles (ASV/USV) and underwater vehicles (AUV/ROV/gliders)
- Simulation environments and sensor modeling
- Control systems, guidance, and autonomy frameworks
- Drivers, interfaces, and communication protocols
- Datasets, tools, and educational resources

## Website

Rendered site: Publish via your deployment workflow, or use `mkdocs serve` for a local preview.

## Index

* [Vehicle Types](docs/vehicle_types.md) - Surface and underwater vehicle platforms
* [Autonomy Stacks](docs/autonomy_stacks.md) - Full-stack frameworks and systems
* [Simulation](docs/simulation.md) - Gazebo, Stonefish, and other simulators
* [Hardware](docs/hardware.md) - Platforms, sensors, and components
* [Software](docs/software.md) - Control, localization, perception, and tools
* [Drivers](docs/drivers.md) - Hardware drivers and interfaces
* [Datasets](docs/datasets.md) - Training data and test collections
* [Education & Tutorials](docs/education_and_tutorials.md) - Learning resources
* [Events](docs/events.md) - Competitions, conferences, and workshops

## Community

This landscape is a community project curated by the ROS Maritime Robotics Working Group.

**Get Involved:**
* **Meetings:** Monthly (see schedule in the [Google Doc](https://docs.google.com/document/d/1Wnddq4xRXR6HF2XFWeejfUGII_hj7DilKrGcFj1qlEA/edit?usp=drive_link))
* **Zulip Chat:** [Marine Robotics channel](https://openrobotics.zulipchat.com/#narrow/channel/526058-Marine-Robotics)
* **Discourse:** [Maritime Robotics Category](https://discourse.ros.org/c/maritime/36)
* **Project Board:** [GitHub Projects](https://github.com/orgs/ros-maritime/projects)

## Contributing

Contributions to the landscape are highly encouraged! From simple fixes to spelling and grammar all the way to adding new sections.

### Quick Changes on GitHub

Simple changes can be made by clicking the "Edit on GitHub" link at the top of any page on the website.

1. Open the page you want to edit
2. Click the "Edit on GitHub" link
3. Make changes in the online editor
4. Create a pull request

### Local Development

For substantial changes, including new pages or images:

1. Clone the repository:
   ```bash
   git clone <repo-url>
   cd maritime_robotic_landscape
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Start local server:
   ```bash
   mkdocs serve
   ```

4. View at `localhost:8000`

5. Make your changes and create a pull request

## License

This documentation is provided under the Creative Commons Attribution-ShareAlike 4.0 International License.
