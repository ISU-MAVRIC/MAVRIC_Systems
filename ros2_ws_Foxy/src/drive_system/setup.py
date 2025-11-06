from setuptools import setup

package_name = 'drive_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mavric',
    maintainer_email='ronsecondmail@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    #Define what the package does/calls upon start up
    #<command-name> = <module-path>:<function-name>
    entry_points={
        'console_scripts': [
            'can_control.py = drive_system.can_control:main',
            'scale_tuning.py = drive_system.scale_tuning:main',
        ],
    },
)
