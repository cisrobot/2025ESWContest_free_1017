from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'patrol_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='haehae_00@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'firestore_bridge = patrol_nodes.firestore_bridge:main',
        'location_data_send.py = patrol_nodes.location_data_send:main',    # 새로 추가된 더미 GPS 발행 노드
        ],
    },
)
