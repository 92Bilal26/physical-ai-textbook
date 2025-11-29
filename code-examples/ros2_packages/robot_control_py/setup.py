from setuptools import find_packages, setup

package_name = 'robot_control_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Learner',
    maintainer_email='learner@physical-ai.local',
    description='ROS 2 Python package demonstrating robot velocity control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = robot_control_py.velocity_controller:main',
        ],
    },
)
