from setuptools import find_packages, setup

package_name = 'robot_a_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hv-05',
    maintainer_email='superfine100@gmail.com',
    description='Robot A map saving and sector coordinate recording',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'map_saver_node = robot_a_mapping.map_saver_node:main',
        ],
    },
)
