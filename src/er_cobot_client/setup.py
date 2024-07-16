from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'er_cobot_client'
submodules = 'er_cobot_client/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonlab02',
    maintainer_email='stargaze221@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_platform_client = er_cobot_client.node_platform_client:main',
            'node_listener = er_cobot_client.node_listener:main',
        ],
    },
)
