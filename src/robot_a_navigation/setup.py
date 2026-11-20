from setuptools import find_packages, setup

package_name = 'robot_a_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hv-05',
    maintainer_email='superfine100@gmail.com',
    description='Robot A Survey Mission Manager (a_mission) · Re-survey (a_remission) · staged test versions and dummies',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'a_mission = robot_a_navigation.a_mission:main',
            'a_remission = robot_a_navigation.a_remission:main',
            'v1_initial_pose = robot_a_navigation.v1_initial_pose:main',
            'v2_capture_send = robot_a_navigation.v2_capture_send:main',
            'v3_mission_cmd = robot_a_navigation.v3_mission_cmd:main',
            'v4_battery = robot_a_navigation.v4_battery:main',
            'dummy_receiver = robot_a_navigation.dummy_receiver:main',
            'dummy_server = robot_a_navigation.dummy_server:main',
            'dummy_battery = robot_a_navigation.dummy_battery:main',
            'dummy_remission_server = robot_a_navigation.dummy_remission_server:main',
        ],
    },
)
