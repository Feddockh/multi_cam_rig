from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_cam_rig'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/multi_cam_rig/config', glob('config/*.yaml')),
        ('share/multi_cam_rig/calibration', glob('calibration/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayden',
    maintainer_email='hayden4feddock@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_gui_node = multi_cam_rig.trigger_gui_node:main',
            'zed_image_node = multi_cam_rig.zed_image_node:main',
            'firefly_image_node = multi_cam_rig.firefly_image_node:main',
        ],
    },
)
