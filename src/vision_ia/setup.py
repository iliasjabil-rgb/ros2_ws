from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_ia'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. Marqueur d'index : INDISPENSABLE pour ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2. Le fichier package.xml
        ('share/' + package_name, ['package.xml']),
        # 3. Les fichiers de lancement (launch files)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='godefroi',
    maintainer_email='godefroi@todo.todo',
    description='Package de vision pour ROS 2',
    license='TODO: License declaration',
    # J'ai remplacé extras_require par une structure plus simple pour éviter les warnings
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                # Le nom à gauche du '=' est celui que ROS utilise pour lancer le script
                'vision_IA = vision_ia.vision_IA:main',
                'camera = vision_ia.camera:main',
            ],
        },

)