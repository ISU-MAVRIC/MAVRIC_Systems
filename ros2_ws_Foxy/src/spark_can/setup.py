import os
from glob import glob
from setuptools import find_packages, setup


package_name = "spark_can"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        # ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/msg", glob("msg/*.msg")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=[
        "setuptools",
        "python-can",
        "numpy",
        "BSON",
        "PyYAML",
        "bitstring"
    ],
    zip_safe=True,
    maintainer="golden",
    maintainer_email="golden@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            #
            # FORMAT:
            #
            # <executable name> = <package_name>.<file_name>:<function_name>
            #                
            # NOTE:
            #                        
            # <executable name> is the name of the executable thats named in the launch file.
            # <package_name> is the name of the package that the executable is associated with.
            # <file_name> is the name of the file that contains the function
            # <function_name> is the name of the function that is called when the executable is run.

            "spark_can_drive_train = spark_can.spark_can_drive_train:main",
            "scale_startups = spark_can.scale_startups:main",

            # '900MHz_Interface = SparkCan.900MHz_Interface:main',
        ],
    },
)
