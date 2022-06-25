# Mobile Robot SADG Controller

Implementation of the receding horizon controller
  presented in

> A. Berndt, N. van Duijkeren, L. Palmieri, A. Kleiner, T. Keviczky, "Receding Horizon Re-ordering of Multi-Agent Execution Schedules", in _Transactions of Robotics_. 

Receding Horizon Feedback Control Scheme |
:-------------------------:| 
![](.github/diagrams/feedback_diagram.svg)
_As AGVs progress, and are inevitably delayed, their status is tracked, and an optimization iteration is performed to re-order the AGVs based on a Finite Horizon SADG Subset calculated by Algorithm 3. The solution of the MILP is used to update the SE-ADG used for plan execution in a feedback loop._ |

Full Maze             |  Half Maze |  Warehouse |  Islands
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
![Full Maze](.github/diagrams/full_maze.svg)  |  ![Half Maze](.github/diagrams/half_maze.svg) | ![Half Maze](.github/diagrams/warehouse.svg) | ![Half Maze](.github/diagrams/islands.svg)



## Get Started

Dependencies:

- [Poetry](https://python-poetry.org/docs/)
- [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 

```bash
git clone git@github.com:alexberndt/mobile-robot-sadg-controller.git
cd mobile-robot-sadg-controller
poetry install
```

#### Install libMultiRobotPlanning

Install build dependencies

```bash
sudo apt-get install g++ cmake libboost-program-options-dev \ 
libyaml-cpp-dev clang-tidy \ 
clang-format doxygen
```

Compile from source
```bash
cd bin/libMultiRobotPlanning/
mkdir build
cd build 
cmake ..
make
```