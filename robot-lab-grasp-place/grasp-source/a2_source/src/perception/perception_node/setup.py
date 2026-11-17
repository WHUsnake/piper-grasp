from setuptools import setup
from glob import glob
setup(name='perception_node', version='0.2.0', packages=['perception_node'],
 data_files=[('share/ament_index/resource_index/packages',['resource/perception_node']),('share/perception_node',['package.xml']),('share/perception_node/launch',glob('launch/*.launch.py')),('share/perception_node/config',glob('config/*.yaml'))],
 entry_points={'console_scripts': ['perception_node=perception_node.node:main']})
