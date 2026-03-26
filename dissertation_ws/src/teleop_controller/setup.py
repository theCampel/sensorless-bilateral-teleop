import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- This is the important part for launch files ---
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # --- This is the important part for config files ---
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.rviz'))),
        # --- URDF files for custom robot descriptions ---
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.com',
    description='Dissertation teleoperation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # --- This tells ROS your Python script is an executable node ---
    entry_points={
        'console_scripts': [
            'teleop_node = teleop_controller.teleop_node:main',
            'force_feedback_node = teleop_controller.force_feedback_node:main',
            'motion_node = teleop_controller.motion_node:main',
            'effort_graph_node = teleop_controller.effort_graph_node:main',
        ],
    },
)