import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'SparkCan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='golden',
    maintainer_email='golden@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    package_data={
        package_name: ['msg/*.msg'],
    },
    entry_points={
        'console_scripts': [
            'SparkCAN_Drive_Train = SparkCan.SparkCAN_Drive_Train:main',
            'Scale_Startups = SparkCan.Scale_Startups:main',
            # '900MHz_Interface = SparkCan.900MHz_Interface:main',
        ],
    },
)
