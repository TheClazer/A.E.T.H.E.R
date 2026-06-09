import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aether_health_cockpit'

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
    description='A.E.T.H.E.R node package: aether_health_cockpit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cockpit_node = aether_health_cockpit.cockpit_node:main',
        ],
    },
)
