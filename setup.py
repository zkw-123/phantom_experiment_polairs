from setuptools import setup
import os
from glob import glob

package_name = 'polaris_ultrasound'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2包，用于集成Polaris跟踪器和超声成像',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polaris_reader = polaris_ultrasound.polaris_reader_node:main',
            'ultrasound_reader = polaris_ultrasound.ultrasound_reader_node:main',
            'calibration_recorder = polaris_ultrasound.calibration_recorder:main',
            'phantom_experiment = polaris_ultrasound.phantom_experiment:main',
        ],
    },
)