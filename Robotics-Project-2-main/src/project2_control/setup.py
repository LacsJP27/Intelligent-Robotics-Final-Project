from setuptools import setup, find_packages

package_name = 'project2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kand0007',
    maintainer_email='kand0007@todo.todo',
    description='Reactive behavior controller (Phase 2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = project2_control.controller_node:main',
        ],
    },
)
