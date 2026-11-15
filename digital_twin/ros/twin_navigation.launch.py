"""Optional Humble SLAM Toolbox + Nav2 on the isolated postal_sim namespace."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent
    # Missing external packages produce a clear launch error; no fallback to oracle poses.
    bringup = Path(get_package_share_directory('nav2_bringup'))
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                namespace='postal_sim', parameters=[str(root/'slam.yaml')],
                remappings=[('/tf', '/postal_sim/tf'), ('/tf_static', '/postal_sim/tf_static')])
    navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/navigation_launch.py')),
        launch_arguments=dict(namespace='postal_sim', use_sim_time='true', autostart='true',
                              use_composition='false', params_file=str(root/'nav2.yaml')).items())
    return LaunchDescription([slam, navigation])
