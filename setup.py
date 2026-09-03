import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rpp_f110'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raul Villavicencio',
    maintainer_email='raulvillaes@outlook.com',
    description='Regulated Pure Pursuit para el F1TENTH del simulador AutoDRIVE',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rpp_node = rpp_f110.rpp_node:main',
            'lap_node = rpp_f110.lap_node:main',
            'path_node = rpp_f110.path_node:main',
        ],
    },
)
