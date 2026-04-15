import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'torso_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pika',
    maintainer_email='shahpawan56@gmail.com',
    description='TIAGo torso controller: optimal height calculation and trajectory execution',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'torso_adjust_server = torso_controller.torso_adjust_action_server:main',
            'test_torso_adjust = torso_controller.test_torso_adjust:main',
        ],
    },
)
