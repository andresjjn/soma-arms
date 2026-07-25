import os
from glob import glob

from setuptools import setup

package_name = 'soma_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'srdf'), glob('srdf/*.srdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andres Jejen',
    maintainer_email='andresjt93@gmail.com',
    description='URDF/xacro description of the SOMA arms and torso lift',
    license='MIT',
    entry_points={'console_scripts': []},
)
