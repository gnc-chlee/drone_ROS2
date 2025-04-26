from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sjcu_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lch',
    maintainer_email='chungh6577@sju.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'move_publisher = sjcu_topic.move_publisher:main',
        'position_subscriber = sjcu_topic.position_subscriber:main',
        'position_control = sjcu_topic.position_control:main',
        'rotate_drone_server = sjcu_topic.rotate_drone_server:main',
        'move_drone_server = sjcu_topic.move_drone_server:main',
        ],
    },
)
