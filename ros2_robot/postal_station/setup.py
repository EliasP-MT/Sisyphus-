from setuptools import setup

setup(name='postal_station', version='0.2.0', packages=['postal_station'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/postal_station']),
                  ('share/postal_station', ['package.xml']),
                  ('share/postal_station/launch', ['launch/station.launch.py']),
                  ('share/postal_station/web', ['web/index.html']),
                  ('share/postal_station/rviz', ['rviz/postal.rviz'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['web = postal_station.web_node:main',
                                        'arm_motion = postal_station.arm_motion:main']})
