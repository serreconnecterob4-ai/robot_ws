from setuptools import setup
import os
from glob import glob

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 🔥 OBLIGATOIRE POUR ROS2 LAUNCH
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toi',
    maintainer_email='toi@mail.com',
    description='Camera package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_control_node = camera.camera_control_node:main',
            'camera_publisher = camera.camera_publisher:main',
            'camera_manager = camera.camera_manager:main',
            'camera_bridge = camera.camera_bridge:main',
            'capture_manager = camera.capture_manager:main',
        ],
    },
)