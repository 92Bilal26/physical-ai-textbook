from setuptools import find_packages, setup

package_name = 'param_example_py'

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
    description='ROS 2 Python package demonstrating parameter server usage',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_node = param_example_py.param_node:main',
        ],
    },
)
