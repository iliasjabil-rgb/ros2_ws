from setuptools import find_packages, setup

package_name = 't16000m_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/t16000m_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilias',
    maintainer_email='iliasjabil@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        't16000m_teleop   = t16000m_teleop.t16000m_teleop_node:main',
        't16000m_debug    = t16000m_teleop.t16000m_debug_node:main',
        'cartesian_teleop = t16000m_teleop.t16000m_cartesian_teleop_node:main',
        ],
    },
)
