import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_serial_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dual_serial_bridges.launch.py']),
        ('share/' + package_name + '/config', ['config/bridges.params.yaml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ilias',
    maintainer_email='iliasjabil@gmail.com',
    description='ROS 2 serial bridges for Arduino MEGA and UNO',
    license='MIT',
   entry_points={
        'console_scripts': [
            'mega_bridge = dual_serial_bridge.mega_bridge:main',
            'mega_driver = dual_serial_bridge.mega_driver:main',
            'uno_bridge  = dual_serial_bridge.uno_bridge:main',
            'uno_driver  = dual_serial_bridge.uno_driver:main',
            'system_monitor = dual_serial_bridge.system_monitor:main',
        ],
    },
)
