from setuptools import find_packages, setup

package_name = 'servo_motor'

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
    description='Lifecycle motor node with per-ID limits using serial service backend.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = servo_motor.motor_node:main',
        ],
    },
)
