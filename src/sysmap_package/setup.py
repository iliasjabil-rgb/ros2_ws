from setuptools import find_packages, setup
import os
from glob import glob

# MODIFICATION ICI :
package_name = 'sysmap_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. On exporte TOUS les fichiers Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # 2. On exporte TOUS les fichiers 3D (URDF)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Controle teleop URDF sysmap',
    license='TODO',
    entry_points={
        'console_scripts': [
            'robot_brain = sysmap_package.robot_brain:main',
            'sysmap_teleop = sysmap_package.manipulator_teleop:main',
            'sysmap_joystick = sysmap_package.sysmap_joystick:main',
            'simple_servo3 = sysmap_package.simple_servo3_test:main',
            'csv_recorder = sysmap_package.csv_recorder:main',
            'recorder_manager = sysmap_package.recorder_manager:main',
        ],
    },
)
