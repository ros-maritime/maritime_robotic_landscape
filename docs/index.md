--8<-- "README.md::4"

## Welcome to Marine Robotics!

Whether you're a curious student, experienced roboticist, or somewhere in between - this site has resources for you:

- **🎓 New to marine robotics?** Start with our [Getting Started Guide](getting_started.md) - no prior experience needed!
- **🔍 Looking for specific packages or tools?** Browse by category using the navigation menu
- **📚 Want to learn?** Check out [Education & Tutorials](education_and_tutorials.md) with hands-on guides
- **❓ Have questions?** See the [FAQ](faq.md) and [Glossary](glossary.md) for quick answers

!!! tip "Quick Start for Beginners"

    Never built a robot before? No problem! You can start learning with free simulators - no hardware or ocean required. See [Getting Started](getting_started.md) to drive your first virtual robot in simulation.

!!! note "Fast Track for Experts"

    **Experienced maritime roboticist?** Jump directly to:

    - **[Expert Reference](expert_reference.md)** - Package comparisons, integration notes, vendor selection criteria
    - **[Operations Guide](operations.md)** - Pre-deployment checklists, power budgets, failure modes, field-tested procedures
    - **[Integration Examples](integration_examples.md)** - Reference architectures, launch files, tf trees, proven sensor suites
    - **Hardware Comparisons** - [DVL](hardware.md#dvl-selection-guide), [INS/IMU](hardware.md#insimu-selection-guide), [Thrusters](hardware.md#thruster-selection-guide), [Budget Planning](hardware.md#budget-planning)
    - **[Known Issues](software.md#known-issues-thrust-allocation)** - Debugging guidance for [thrust allocation](software.md#known-issues-thrust-allocation), [localization](software.md#known-issues-localization), [acoustic communication](software.md#known-issues-acoustic-communication)

!!! info "Contributions Welcome"

    This landscape is under active development and we welcome contributions! You can submit a pull request with your changes or open an issue on GitHub. Join our monthly meetings to discuss maritime robotics development with the community.

## Community

This landscape is a community project maintained by the ROS Maritime Robotics Working Group. The best way to contribute is to join our monthly meetings and help expand and improve this resource.

**Main repository:** [Maritime Robotics Working Group](https://github.com/ros-maritime/community)

### Meetings

The Maritime Robotics Working Group meets regularly; meetings are announced on the [ROS Discourse Maritime Robotics category](https://discourse.ros.org/c/maritime/36), and recordings and meeting notes are posted after each meeting.

* **Meeting agendas and minutes:** [Google Doc](https://docs.google.com/document/d/1Wnddq4xRXR6HF2XFWeejfUGII_hj7DilKrGcFj1qlEA/edit?usp=drive_link)
* **Meeting calendar invites:** [Google Group](https://groups.google.com/g/maritime-robotics-working-group-invites)

### Team / Community Coordination

* **Instant Messaging:** [Matrix chat #ros-maritime](https://matrix.to/#/#ros-maritime:matrix.org)
* **Forums:** [ROS Discourse Maritime Robotics Category](https://discourse.ros.org/c/maritime/36)
* **Project Tracking:** [GitHub Project Board](https://github.com/orgs/ros-maritime/projects)

## Contributing

Contributions to the landscape are highly encouraged, from simple fixes to spelling and grammar all the way to adding new sections to the landscape.

### Quick Changes in GitHub

Simple changes to existing pages can be made directly through GitHub using the "Edit on GitHub" link at the top of every page.

**To edit an existing page:**

1. Open the page you want to edit.
2. Click the "Edit on GitHub" link at the top of the page.
3. Make your changes in the online editor.
4. Create a separate branch when prompted and submit a pull request.

The team will review your contribution and either provide feedback or merge your changes to update the website.

### Changes Using Git (New Pages and Images)

More substantial changes, including adding new pages or modifying images, are best done locally. For these changes, use the standard git workflow:

1. Clone the repository: `git clone <repo-url>`
2. Modify the documentation as needed (add, change, or delete content).
3. Test locally with [mkdocs](https://www.mkdocs.org/): `mkdocs serve` to preview changes at `localhost:8000`
4. Create a branch for your changes and submit a pull request.

--8<-- "docs/goatcounter.html"
