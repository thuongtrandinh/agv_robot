from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_trajectory_tracking'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install results directory (empty initially, will be created if not exists)
        (os.path.join('share', package_name, 'results'), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thuong',
    maintainer_email='thuong.trandinh@hcmut.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fuzzy_trajectory_controller = agv_trajectory_tracking.fuzzy_trajectory_controller:main',
            'trajectory_publisher = agv_trajectory_tracking.trajectory_publisher:main',
            'trajectory_visualizer = agv_trajectory_tracking.trajectory_visualizer:main',
            'trajectory_plotter = agv_trajectory_tracking.trajectory_plotter:main'
        ],
    },
)
