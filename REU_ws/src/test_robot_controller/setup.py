from setuptools import find_packages, setup

package_name = 'test_robot_controller'

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
            "test_node = test_robot_controller.my_first_node:main",
            "draw_circle = test_robot_controller.draw_circle:main",
            "pose_subscriber = test_robot_controller.pose_subscriber:main"
        ],
    },
)
