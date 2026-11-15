import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

HERE = os.path.dirname(os.path.abspath(__file__))
MORPH = os.path.dirname(os.path.dirname(HERE))


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', os.path.join(MORPH, 'ros2', 'a_morph_state.py')], output='screen'),
        ExecuteProcess(cmd=['python3', os.path.join(MORPH, 'ros2', 'a_morph_command.py')], output='screen'),
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'a_morph_dynamic.py')], output='screen'),
    ])
