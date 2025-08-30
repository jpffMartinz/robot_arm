from setuptools import find_packages, setup

package_name = 'arm_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noiyuh',
    maintainer_email='1201043@isep.ipp.pt',
    description='Robot arm driver for ak80-8 actuators',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'arm_node = arm_interface.arm_node:main',
        ],
    },
)
