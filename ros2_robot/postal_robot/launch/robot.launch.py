"""Embedded bring-up: core (single serial owner), two cameras, robot_state_publisher."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_DIR = os.path.expanduser('~/postal_config')


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    params = get('params')
    description = Path(get('description')).read_text()
    result = [
        Node(package='postal_robot', executable='core', namespace='robot', name='core', output='screen',
             parameters=[params, {'calibration': get('calibration'), 'poses': get('poses')}],
             on_exit=Shutdown(reason='core exited; systemd restarts the whole runtime')),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='robot',
             output='screen', parameters=[{'robot_description': description}]),
    ]
    if get('cameras') == 'true':
        for index in (1, 2):
            result.append(Node(package='postal_robot', executable='usb_camera', name='camera',
                               namespace=f'robot/camera_{index}', output='screen',
                               parameters=[params], respawn=True, respawn_delay=3.))
    return result


def generate_launch_description():
    share = get_package_share_directory('postal_robot')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'robot.yaml')),
        DeclareLaunchArgument('calibration', default_value=os.path.join(CONFIG_DIR, 'calibration.local.json')),
        DeclareLaunchArgument('poses', default_value=os.path.join(CONFIG_DIR, 'poses.local.json')),
        DeclareLaunchArgument('description', default_value=os.path.join(CONFIG_DIR, 'elrobot.urdf')),
        DeclareLaunchArgument('cameras', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
