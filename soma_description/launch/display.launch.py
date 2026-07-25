"""Show the SOMA model in RViz with a slider per joint.

  ros2 launch soma_description display.launch.py
  ros2 launch soma_description display.launch.py model:=single_arm.urdf.xacro
  ros2 launch soma_description display.launch.py rviz:=false

Nothing here touches hardware: it publishes a robot description and lets
joint_state_publisher_gui move the model. Driving real servos is the job of
soma_driver, and it needs its own explicit arming.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('soma_description')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='soma_bench.urdf.xacro',
        description='xacro file inside urdf/ to visualise',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz2'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Open the joint slider GUI'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('model')]),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        gui_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # Sliders to move each joint by hand (mimic joints follow on their own)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'soma.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
