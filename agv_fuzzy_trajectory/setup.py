from setuptools import setup
import os
from glob import glob

package_name = 'agv_fuzzy_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thuong',
    maintainer_email='thuong.trandinh@hcmut.edu.vn',
    description='Fuzzy logic trajectory tracking controller for AGV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fuzzy_trajectory_controller = agv_fuzzy_trajectory.fuzzy_trajectory_controller:main'
        ],
    },
)
