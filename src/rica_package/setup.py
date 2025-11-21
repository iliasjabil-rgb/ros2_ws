from setuptools import setup
from glob import glob
import os

package_name = 'rica_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share','ament_index','resource_index','packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),  # <-- AJOUT
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilias',
    maintainer_email='you@example.com',
    description='RICA control and visualization (ament_python)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rica_bridge = rica_package.rica_bridge:main',
            'diff_drive_sim = rica_package.diff_drive_sim:main',
            'joint_teleop  = rica_package.joint_teleop:main',
        ],
    },
)
