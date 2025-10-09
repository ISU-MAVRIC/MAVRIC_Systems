**See ros2_ws_Foxy for current development space and example of the following**

**General Workspace Structure**
Workspace
	build
		Temporary build artifacts. Made by building tools and do not need to tampered with.
	install
		Final "installed" versions of all your packages
	log
		Logs from builds and runtime - useful for debugging
	src
		package - launch
			launch
				.launch.py files
			package.xml
			CMakeList.txt
		package - msg
			msg
				.msg files
			package.xml
			CMakeList.txt
		package - with Python
			package_name
				node
				node
				script
			resource
				empty
			package.xml
			setup.cfg
			setup.py
		package - with C++
			src
				node
				node
				script
			package.xml
			CMakeList.txt
			include
				C++ header files if you have any, otherwise don't need include folder
		package - with C++ and Python (not recommended)
			src
				C++ sources
			include
				C++ headers
			package_name
				Python modules
			package.xml 
			CMakeLists.txt
			{setup.py, setup.cfg, resource} (optional: don't need if python is just helper scripts)


**Packages** - Each "functional group" of ROS2 nodes should be bundled into packages. Examples might include a drive_system package containing steer_train, drive_train, and can_control nodes. Each includes a package.xml that describes the ROS2 specific dependencies of the package (doesn't include C++ or Python dependencies).

**Nodes** - Smallest unit of computation is ROS2 structure, does one thing for the system

**Scripts** - Similar to nodes but not using Node class. Generally helper scripts for nodes in the same package to use.

**Python-only packages** - soft requirement to have setup.py, setup.cfg, and /resource. setup.py tells system how to use the python package (installation/inking), setup.cfg adds small configurations, and /resource allows ROS2 to recognize this python package. Source files are kept in a folder named after the package itself, which is required for Python. You can just Cmake instead of a setup.py, but you end up using setup.py in the CMake itself, and it is unconventional. This could be done for blending C++ and Python in one package, or to use CMake consistently throughout the system. 

**C++ packages** - hard requirement to have CMakeList.txt (compiling/linking). Do not need /resource because CMake indexes packages for ROS2 automatically. Source files are kept in src per C++ convention. You could use package_name instead of src like Python does, but you would need to be careful with your CMakeLists (not that hard) and it is not convention to unify when using both python and C++ packages.


**mavric_msg and mavric_launch packages** - create msg with CMake instead of setup.py for simpler building. Don't need python build tools unless using a python package, which we likely will not be in the msg package. We might in the launch package. Basic .launch.py scripts for launching do not need setup.py, but it could be nice to use setup.py for launching in the future if we do use more advanced python.
	**Naming**: you cannot call a package "launch" because it overwrites a ROS2 package. You may not have matching package name and subdirectory name in your message directory, because it breaks the internal C++ namespace handling. I chose to change the package to mavric_msg and leave the internal subdirectory that holds .msg files as "msg". Package names are case-insensitive, so you may not use "Launch" or Msg.msg either.
