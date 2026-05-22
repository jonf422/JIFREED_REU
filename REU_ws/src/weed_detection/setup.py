from setuptools import find_packages, setup

package_name = 'weed_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "realsense_node = weed_detection.realsense_node:main",
            "realsense_display_node = wed_detection.realsense_display_node:main",
            "arducam_node = weed_detection.arducam_node:main"
        ],
    },
)
