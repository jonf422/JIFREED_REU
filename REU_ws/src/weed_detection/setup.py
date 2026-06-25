from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'weed_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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

            #Vision
            "realsense_multistream_node = weed_detection.vision.realsense_multistream_node:main",
            "arducam_node = weed_detection.vision.arducam_node:main",

            "realsense_node = weed_detection.vision.realsense_node:main",

            #Control
            "drive_controller_node = weed_detection.control.drive_controller_node:main",

            #Navigation
            "wheel_odom_node = weed_detection.navigation.wheel_odom_node:main",

            #Display
            "combined_display_node = weed_detection.display.combined_display_node:main",

            "arducam_display_node = weed_detection.display.arducam_display_node:main",
            "realsense_display_node = weed_detection.display.realsense_display_node:main",
            "realsense_depth_display_node = weed_detection.display.realsense_depth_display_node:main",

            #Calibration
            "imu_noise_cali = weed_detection.calibration.imu_noise_cali:main" 
            
        ],
    },
)
