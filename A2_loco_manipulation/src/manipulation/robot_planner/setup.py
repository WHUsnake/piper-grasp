from setuptools import setup
from glob import glob
setup(name='robot_planner', version='0.2.0', packages=['robot_planner'],
 data_files=[('share/ament_index/resource_index/packages',['resource/robot_planner']),('share/robot_planner',['package.xml']),('share/robot_planner/launch',glob('launch/*.launch.py')),('share/robot_planner/config',glob('config/*.yaml'))],
 entry_points={'console_scripts': ['robot_planner=robot_planner.node:main']})
