from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleop_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rgnp',
    maintainer_email='rzghosh@gmail.com',
    description='RC input handler: maps RCIn to Twist setpoints and control mode.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_interface_node = teleop_interface.teleop_interface:main',
            'api_node = teleop_interface.api_node:main',
        ],
    },
)
