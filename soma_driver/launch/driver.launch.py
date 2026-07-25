"""Start the SOMA driver node.

Safe by default:
  ros2 launch soma_driver driver.launch.py

Real hardware (still disarmed until you call the arm service):
  ros2 launch soma_driver driver.launch.py allow_real:=true
  ros2 service call /soma/arm std_srvs/srv/SetBool "{data: true}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    allow_real_arg = DeclareLaunchArgument(
        'allow_real',
        default_value='false',
        description='Permit the arm service to switch to the real PCA9685',
    )
    i2c_arg = DeclareLaunchArgument(
        'i2c_address', default_value='64', description='PCA9685 I2C address (64 = 0x40)'
    )

    return LaunchDescription([
        allow_real_arg,
        i2c_arg,
        Node(
            package='soma_driver',
            executable='arm_controller',
            name='soma_driver',
            output='screen',
            parameters=[{
                'allow_real': LaunchConfiguration('allow_real'),
                'i2c_address': LaunchConfiguration('i2c_address'),
            }],
        ),
    ])
