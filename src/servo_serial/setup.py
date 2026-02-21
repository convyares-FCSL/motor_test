from setuptools import find_packages, setup

package_name = 'servo_serial'

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
    maintainer='motor_test',
    maintainer_email='you@example.com',
    description='Lifecycle node for serial comms with Waveshare ST/SC servos.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = servo_serial.serial_node:main',
        ],
    },
)
