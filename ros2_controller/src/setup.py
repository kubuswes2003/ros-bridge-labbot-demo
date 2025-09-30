from setuptools import setup

package_n = 'robot_controller'

setup(
    name=package_n,
    version='1.0.0',
    packages=[package_n],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_n]),
        ('share/' + package_n, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS2 robot controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = robot_controller.controller_node:main',
            'simple_web_api = robot_controller.simple_web_api:main',
        ],
    },
)