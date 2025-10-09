from setuptools import setup, find_packages
#find_packages() automatically checks full directory and subdirectories for python packages (not just top level).

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['utils', 'utils.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mavric',
    maintainer_email='ronsecondmail@gmail.com',
    description='Multi-use utility package containing scripts used across MAVRIC\'s system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
