from glob import glob
from setuptools import find_packages, setup

package_name = 'servo_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('servo_bridge/web/*')),
    ],
    install_requires=['setuptools', 'aiohttp'],
    zip_safe=True,
    maintainer='motor_test',
    maintainer_email='you@example.com',
    description='ROS-web bridge for servo slider UI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_bridge_node = servo_bridge.ros_bridge_node:main',
        ],
    },
)
