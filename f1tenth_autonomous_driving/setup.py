from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_autonomous_driving'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous driving package for F1TENTH vehicles using LiDAR data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_driver = f1tenth_autonomous_driving.autonomous_driver:main',
        ],
    },
) 