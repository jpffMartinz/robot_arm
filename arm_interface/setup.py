from setuptools import setup
import os
from glob import glob

package_name = 'arm_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noiyuh',
    maintainer_email='1201043@isep.ipp.pt',
    description='STM32 Arm Interface - Bidirectional communication node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge = arm_interface.stm32_bridge:main',
        ],
    },
)
