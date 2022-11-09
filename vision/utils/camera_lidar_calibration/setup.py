from setuptools import setup
import os
from glob import glob

package_name = 'camera_lidar_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=["camera_lidar_calibration", "camera_lidar_calibration.cameraCalibration.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='regatte',
    maintainer_email='ashok_a380@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[
        'pytest',
        'requests'
        ],
    entry_points={
        'console_scripts': [
            'cameracalibrator = camera_lidar_calibration.cameraCalibration.nodes.cameracalibrator:main',
            'cameracheck = camera_lidar_calibration.cameraCalibration.nodes.cameracheck:main',
        ],
    },
)
