import os
from glob import glob

from setuptools import setup

package_name = 'soma_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andres Jejen',
    maintainer_email='andresjt93@gmail.com',
    description='PCA9685 driver (mock and real) for the SOMA arms and torso lift',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_controller = soma_driver.arm_controller_node:main',
            'soma_primitives = soma_driver.primitives_cli:main',
            'soma_sign_check = soma_driver.sign_check_cli:main',
        ],
    },
)
