from setuptools import setup
from glob import glob
import os

package_name = 'mxck_ftg_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omar',
    maintainer_email='omar@example.com',
    description='Stage 3 planner package for MXCarkit FTG.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ftg_planner_node = mxck_ftg_planner.ftg_planner_node:main',
        ],
    },
)