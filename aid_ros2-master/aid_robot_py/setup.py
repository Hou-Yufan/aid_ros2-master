from setuptools import setup
import os
from glob import glob
package_name = 'aid_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('aid_robot_py/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='box',
    maintainer_email='chendongfang@turingvideo.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_manager_node = aid_robot_py.launch_manager:main',
            'map_manager_node = aid_robot_py.map_manager_server:main',
            'ai_manager_node = aid_robot_py.ai_manager_server:main',
            'map_transform_node = aid_robot_py.map_transform:main',
            'robot_pose_pub_node = aid_robot_py.robot_pose_pub:main',
            'robot_status_manager_node = aid_robot_py.robot_status_manager:main',
            'follow_waypionts_transform = aid_robot_py.follow_waypionts_transform:main',
        ],
    },
)
