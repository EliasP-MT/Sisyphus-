from setuptools import setup

setup(name='postal_robot', version='0.2.0', packages=['postal_robot'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/postal_robot']),
                  ('share/postal_robot', ['package.xml']),
                  ('share/postal_robot/launch', ['launch/robot.launch.py']),
                  ('share/postal_robot/config', ['config/robot.yaml'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['core = postal_robot.core_node:main',
                                        'usb_camera = postal_robot.camera:main']})
