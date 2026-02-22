from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'data_logging'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bag_recorder_launch.py',
        ])
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fadag',
    maintainer_email='fada@uw.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bag_recorder = data_logging.bag_recorder:main',
        ],
    },
)
