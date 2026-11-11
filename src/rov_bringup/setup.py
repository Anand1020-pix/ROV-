from setuptools import setup
from glob import glob
import os

package_name = 'rov_bringup'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROV Project',
    maintainer_email='maintainer@example.com',
    description='ROV bringup launch files',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'seabed_map_publisher = rov_bringup.seabed_map_publisher:main',
        ],
    },
)
