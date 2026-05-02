from setuptools import setup
import os
from glob import glob

package_name = 'carla_sac_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA SAC Team',
    maintainer_email='user@example.com',
    description='CARLA SAC Training Bridge with Guidelines',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_bridge = carla_sac_bridge.carla_bridge_node:main',
            'camera_monitor = carla_sac_bridge.camera_monitor_node:main',
            'training_controller = carla_sac_bridge.training_controller_node:main',
        ],
    },
)
