import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drip_pan_environment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-hill',
    maintainer_email='davidhil@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_environment = drip_pan_environment.load_environment:main',
            'arm_scene_interface = drip_pan_environment.arm_scene_interface:main',
        ],
    },
)
