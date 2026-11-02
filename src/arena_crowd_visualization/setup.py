from setuptools import setup, find_packages
setup(name='arena_crowd_visualization', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/arena_crowd_visualization']),
 ('share/arena_crowd_visualization',['package.xml'])], install_requires=['setuptools'],
 entry_points={'console_scripts':['crowd_visualizer=arena_crowd_visualization.node:main']})
