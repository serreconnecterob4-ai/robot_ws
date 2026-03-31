from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_pkg'

# Collect all files under a directory tree, preserving sub-folder structure.
def package_files(directory):
    paths = []
    for (path, _directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append((
                os.path.join('share', package_name, path),
                [os.path.join(path, filename)],
            ))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'src', 'description'), glob(os.path.join('src', 'description', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pierre-louis',
    maintainer_email='pierre-louis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_waypoints = navigation_pkg.send_waypoints:main',
            'waypoint_action_server = navigation_pkg.waypoint_action_server:main',
        ],
    },
)
