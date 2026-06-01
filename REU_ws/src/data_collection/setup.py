from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonf4',
    maintainer_email='jonf422@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "teleop_node = data_collection.teleop_node:main",
            "realsense_node = data_collection.realsense_node:main",
            "arducam_node = data_collection.arducam_node:main",
            "data_save = data_collection.data_collection_node:main"
        ],
    },
)
