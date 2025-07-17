from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'econ_image_mosic'

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
    maintainer='hycon',
    maintainer_email='jh.jung@hyconsfot.com',
    description='ROS2 package for creating image mosaic from 4 camera feeds with hardware acceleration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_mosaic_node = econ_image_mosic.image_mosaic_node:main',
            'image_mosaic_hw_node = econ_image_mosic.image_mosaic_hw_node:main',
        ],
    },
)
