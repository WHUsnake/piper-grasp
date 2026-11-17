from setuptools import setup
from glob import glob
setup(name='grasp_generation', version='0.2.0', packages=['grasp_generation'],
 data_files=[('share/ament_index/resource_index/packages',['resource/grasp_generation']),('share/grasp_generation',['package.xml']),('share/grasp_generation/launch',glob('launch/*.launch.py')),('share/grasp_generation/config',glob('config/*.yaml')+glob('config/*.json'))],
 entry_points={'console_scripts': ['grasp_generator=grasp_generation.node:main']})
