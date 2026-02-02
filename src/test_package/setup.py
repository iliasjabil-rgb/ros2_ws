from setuptools import find_packages, setup

package_name = 'test_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilias',
    maintainer_email='iliasjabil@gmail.com',
    description='Noeuds de test pour moteurs, relais et capteurs (INA260, MCP9808, LSM6DS33, VL53L0X).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Capteur de courant INA260
            'current_sensor_test = test_package.current_sensor_test_node:main',

            # Température MCP9808
            'temperature_sensor_test = test_package.temperature_sensor_test_node:main',

            # IMU LSM6DS33
            'gyro_test = test_package.gyro_test_node:main',

            # Distance VL53L0X
            'distance_test = test_package.distance_test_node:main',

            # Moteur DC
            'dc_motor_test = test_package.dc_motor_test_node:main',

            # Servomoteur
            'servo_test = test_package.servo_test_node:main',

            # Relais
            'relay_test = test_package.relay_test_node:main',

            # Stepper via Mega
            'mega_stepper_test = test_package.stepper_test_node:main',
        ],
    },
)
