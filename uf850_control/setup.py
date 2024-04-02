from setuptools import find_packages, setup

package_name = 'uf850_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mover = uf850_control.uf850_mover:main',
            'cleaner = uf850_control.uf850_cleaner:main'
        ],
    },
)
