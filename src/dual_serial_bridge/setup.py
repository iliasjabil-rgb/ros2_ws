from setuptools import setup

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
            'uno_bridge  = dual_serial_bridge.uno_bridge:main',
        ],
    },
)
