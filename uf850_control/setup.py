from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uf850_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-hill',
    maintainer_email='davidhil@andrew.cmu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_simulator = uf850_control.kinematics_simulator:main',
            'mover = uf850_control.uf850_mover:main'
        ],
    },
)
