from setuptools import setup

setup(name='postal_robot', version='0.1.0', packages=['postal_robot'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/postal_robot']),
                  ('share/postal_robot', ['package.xml'])],
      entry_points={'console_scripts': ['motor_feedback = postal_robot.feedback:main',
                                        'usb_camera = postal_robot.camera:main']})
