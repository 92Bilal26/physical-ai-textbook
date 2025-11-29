from setuptools import find_packages, setup

package_name = 'action_example_py'

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
    description='ROS 2 Python package demonstrating action pattern',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_action_server = action_example_py.move_action_server:main',
            'move_action_client = action_example_py.move_action_client:main',
        ],
    },
)
