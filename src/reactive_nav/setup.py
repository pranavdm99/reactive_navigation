from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reactive_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world')) + glob(os.path.join('worlds', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranavdm',
    maintainer_email='pranavdm@umd.edu',
    description='Reactive navigation with teleoperation and autonomous behavior',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = reactive_nav.teleop_node:main',
            'autonomous_nav_node = reactive_nav.autonomous_nav_node:main',
        ],
    },
)
