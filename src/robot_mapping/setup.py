from setuptools import setup

package_name = 'robot_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapping.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    entry_points={
        'console_scripts': [
            'slam_pose_recorder = robot_mapping.slam_pose_recorder:main',
            'pos_logger = robot_mapping.logger:main',
        ],
    },
)
