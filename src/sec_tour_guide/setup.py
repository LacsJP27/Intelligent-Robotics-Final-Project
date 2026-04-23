from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sec_tour_guide'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'models', 'person_cylinder'),
            glob('worlds/models/person_cylinder/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joseph Lacsamana',
    maintainer_email='joseph.p.lacsamana-1@ou.edu',
    description='SEC Robot Tour Guide — autonomous tour guide for Sarkeys Energy Center using TurtleBot 4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
            'tour_state_machine = sec_tour_guide.tour_state_machine:main',
            'safety_monitor = sec_tour_guide.safety_monitor:main',
        ],
},
)
