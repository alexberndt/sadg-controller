#!/bin/bash

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

echo ""
echo "Usage instruction:"
echo "  ros2 launch sadg_controller scenario.launch.xml \ "
echo "            agent_count:=<number of agents> \ "
echo "            roadmap:={test, cover, full_maze, half_maze, islands, warehouse} \ " 
echo "            ecbs_sub_factor:=<ecbs suboptimality factor> \ "
echo "            visualize_sadg:={True, False}"
echo ""
echo "For example (using some defaults):"
echo "  ros2 launch sadg_controller scenario.launch.xml agent_count:=8 roadmap:=test visualize_sadg:=True"
echo ""

exec "$@"
