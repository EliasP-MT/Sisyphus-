"""PC station: web UI + ROS bridge, arm planning bridge, plan preview, RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    urdf = Path(get('description')).read_text().replace(
        'filename="assets/', 'filename="file:///opt/postal_description/assets/')
    planner = get('planner_url')
    share = get_package_share_directory('postal_station')
    return [
        Node(package='postal_station', executable='web', namespace='station', name='web', output='screen',
             parameters=[{'port': int(get('port')), 'planner_url': planner}]),
        Node(package='postal_station', executable='arm_motion', namespace='station', name='arm_motion',
             output='screen', parameters=[{'planner_url': planner}]),
        # Plan preview: a second robot model under the "preview/" TF prefix.
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='station/preview',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'preview/'}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='preview_anchor',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'preview/base_link']),
        Node(package='rviz2', executable='rviz2', arguments=['-d', f'{share}/rviz/postal.rviz'],
             condition=IfCondition(get('rviz')), output='log'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('description', default_value='/opt/postal_description/elrobot_follower.urdf'),
        DeclareLaunchArgument('planner_url', default_value='http://127.0.0.1:8893'),
        DeclareLaunchArgument('port', default_value='8895'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
