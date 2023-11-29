from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

# Inspired by https://github.com/ros2/launch/issues/499
def generate_swarm_launch_descriptions(context: LaunchContext):
    return [
        LaunchDescription([
        Node(
            package='sadg_controller',
            executable='agent',
            output='screen',
            name='agent{}'.format(n),
            parameters=[{
                "agent_ns": 'agent{}'.format(n),
                "uuid": str(uuid4()),
            }]
        )
    ]) for n in range(int(context.launch_configurations['agent_count']))
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "agent_count", default_value=TextSubstitution(text="8")
        ),
        OpaqueFunction(function=generate_swarm_launch_descriptions),
    ])

