from setuptools import setup
from glob import glob
import os

package_name = 'my_waver_app'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chotaehyun',
    maintainer_email='example@example.com',
    description='Waypoint patrol node for UGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_waypoint_node = my_waver_app.patrol_waypoint_node:main',
        ],
    },
)
