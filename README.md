# Mobile Robot SADG Controller

Implementation of the receding horizon controller
  presented in [1](https://arxiv.org/pdf/2010.05254.pdf)

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