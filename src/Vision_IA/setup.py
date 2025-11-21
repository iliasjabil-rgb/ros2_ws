from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'Vision_IA'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='godefroi',
    maintainer_email='godefroi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_IA = Vision_IA.vision_IA:main',
            'camera = Vision_IA.camera:main',
        ],
    },

    data_files=[
            
            ('share/' + package_name, ['package.xml']),
            
            ('share/' + package_name + '/launch', glob('launch/*.py')),
        ],
        
)
