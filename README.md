# SADG Controller

SADG-controller is a Python-library
  containing an implementation of the
  Switchable Action Dependency Graph (SADG)
  receding horizon controller (RHC) feedback scheme
  presented in

> A. Berndt, N. van Duijkeren, L. Palmieri, A. Kleiner, T. Keviczky, "Receding Horizon Re-ordering of Multi-Agent Execution Schedules", currently under review for publication in _Transactions of Robotics_.

SADG Receding Horizon Feedback Control Scheme | Typical MAPF Execution Schemes |
:-------------------------:|:-------------------------:|
![](.github/diagrams/feedback_diagram.svg)| ![](.github/diagrams/typical_mapf_scheme.svg) |

_Our approach significantly reduces the cumulative route completion of agents subjected to large delays by optimizing the ordering of agents based on their progress in a receding horizon fashion, while maintaining collision- and deadlock-free plan execution guarantees._

 Switchable Action Dependency Graph | Roadmap |
:-------------------------:|:-------------------------:|
![](.github/animations/sadg.gif) | ![](.github/animations/roadmap.gif) |

## List of Functionalities

1. Python-based interface for interacting with MAPF planners from [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning)
2. SADG Receding Horizon feedback control scheme implementation

## Installation Instructions

This repository supports several workflows to build and execute the software.
We present here the typical ROS2 approach that will also prepare the dependency [`libMultiRobotPlanning`](https://github.com/whoenig/libMultiRobotPlanning) automatically.

### Install ROS2

We currently support ROS2 Galactic and ROS2 Humble, the former is advised for Ubuntu 20.04 users and the latter for For Ubuntu 22.04 users:
- [ROS Galactic](https://docs.ros.org/en/galactic/Installation.html)
- [ROS Humble](https://docs.ros.org/en/humble/Installation.html)

### Prepare workspace
*FOR TRo REVIEWERS: section is currently not relevant, unzip the attached workspace and navigate to it using `cd`*

Create a workspace (e.g., in your home folder) and clone `sadg-controller`.
```bash
mkdir -p ~/sadg_ws/src
cd ~/sadg_ws/src
git clone --recurse-submodules git@github.com:<organization>/<repository>.git
cd ~/sadg_ws
```

### Install dependencies
From `~/sadg_ws`, execute:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build workspace
From `~/sadg_ws`, execute:
```bash
colcon build --symlink-install  # --symlink-install is optional
```

## Examples

To start a simulation, run the following

#### Terminal 1: Initialize the Agents
```bash
source install/setup.bash
ros2 launch sadg_controller n8_agents.launch.xml
```

#### Terminal 2: Start the Controller
```bash
source install/setup.bash
ros2 launch sadg_controller n8_controller.launch.xml
```

#### Terminal 3: Visualize the Plan Execution
```bash
source install/setup.bash
ros2 launch sadg_controller n8_simulation.launch.xml
```

<!-- #### Terminal 4: Visualize the SADG
```bash
source devel/setup.zsh
roslaunch launch/8/sadg.launch
``` -->

Full Maze             |  Half Maze |  Warehouse |  Islands
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
![Full Maze](.github/animations/full_maze.gif)  |  ![Half Maze](.github/animations/half_maze.gif) | ![Half Maze](.github/animations/warehouse.gif) | ![Half Maze](.github/animations/islands.gif)

## Contribute

To contribute to this repository, set up your development environment as follows

1. Follow the [installation instructions](#installation-instruction)
2. Set up pre-commit hooks:
    ```bash
    poetry run pre-commit install
    ```
