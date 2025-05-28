from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'drone_project'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉토리와 파일 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'camera_display = drone_project.camera_display:main',
            'turtlebot_mover = drone_project.turtlebot_mover:main',
            'turtlebot_control = drone_project.turtlebot_control:main',
        ],
    },
)
