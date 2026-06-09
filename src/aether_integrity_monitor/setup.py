import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aether_integrity_monitor'

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
    maintainer='TheClazer',
    maintainer_email='therayyn16@gmail.com',
    description='A.E.T.H.E.R node package: aether_integrity_monitor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = aether_integrity_monitor.monitor_node:main',
        ],
    },
)
