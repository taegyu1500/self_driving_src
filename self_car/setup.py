from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'self_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='11306260+liangfuyuan@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'self_drive = self_car.self_drive:main',
            'drive_state = self_car.state:main',
            'tmd = self_car.twistMiddleware:main',
            'lane_detection = self_car.lane_detection:main',
            'new_drive = self_car.new_self_drive:main',
        ],
    },
)
