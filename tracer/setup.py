from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'tracer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'tracer', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'tracer', 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deen',
    maintainer_email='addeenmahbub@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_controller = tracer.drone_controller:main',
            'car_driver = tracer.car_driver:main',
            'cam_stabiliser = tracer.cam_stabiliser:main',
            'detector = tracer.detector:main',
        ],
    },
)
