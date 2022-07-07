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

## List of Functionalities

1. Python-based interface for interacting with MAPF planners from [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning)
2. SADG Receding Horizon feedback control scheme implementation

## Installation Instructions

First, make sure you have installed:

- [Poetry](https://python-poetry.org/docs/) for consistent python dependency management
- [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) for asynchronous inter-agent communication

```bash
git clone --recurse-submodules git@github.com:alexberndt/sadg-controller.git
cd sadg-controller
make
```

> If `make` returns an error, make sure the following dependencies are installed:
```bash
sudo apt-get install g++ cmake libboost-program-options-dev \
libyaml-cpp-dev clang-tidy  clang-format doxygen
```

> This repo contains [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning), a library which contains `MAPF` planners such as `CBS` and `ECBS` used by the SADG feedback scheme. libMultiRobotPlanning is compiled from source when running `make`.

## Examples

Full Maze             |  Half Maze |  Warehouse |  Islands
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
![Full Maze](.github/diagrams/full_maze.svg)  |  ![Half Maze](.github/diagrams/half_maze.svg) | ![Half Maze](.github/diagrams/warehouse.svg) | ![Half Maze](.github/diagrams/islands.svg)


## Contribute

To contribute to this repository, set up your development environment as follows

1. Follow the [installation instructions](#installation-instruction)
2. Set up pre-commit hooks:
    ```bash
    poetry run pre-commit install
    ```
