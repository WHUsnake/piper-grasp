from setuptools import setup
from glob import glob
setup(name='task_executor', version='0.2.0', packages=['task_executor'],
 data_files=[('share/ament_index/resource_index/packages',['resource/task_executor']),('share/task_executor',['package.xml']),('share/task_executor/launch',glob('launch/*.launch.py')),('share/task_executor/config',glob('config/*.yaml'))],
 entry_points={'console_scripts': ['task_executor=task_executor.node:main']})
