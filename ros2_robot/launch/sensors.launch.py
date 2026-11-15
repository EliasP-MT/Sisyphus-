"""Read-only hardware bringup with one serial owner and two cameras."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration', description='Absolute path to verified calibration JSON'),
        Node(package='postal_robot', executable='motor_feedback', output='screen',
             parameters=[{'calibration': LaunchConfiguration('calibration')}]),
        Node(package='postal_robot', executable='usb_camera', name='camera', namespace='/robot/camera_1',
             output='screen', parameters=[{
                 'device': '/dev/v4l/by-id/usb-GENERAL_GENERAL_WEBCAM_JH0319_20210712_v102-video-index0',
                 'frame_id': 'camera_1_optical_frame'}]),
        Node(package='postal_robot', executable='usb_camera', name='camera', namespace='/robot/camera_2',
             output='screen', parameters=[{
                 'device': '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index0',
                 'frame_id': 'camera_2_optical_frame'}]),
    ])
