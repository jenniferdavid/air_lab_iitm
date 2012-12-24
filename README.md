air_lab_iitm
========================
This repository contains source code for [ROS](http://www.ros.org) stack which is aimed at enhancing productivity while working with MobileRobots.

License
-------
[GNU GPL v3.0](http://www.gnu.org/licenses/gpl-3.0.txt)

Release Notes
-------------
Version: 0.1

Stack Name 
----------------
air_lab_iitm

Stack Details
-------------------
A stack for working with MobileRobots. This stack includes the following packages:
* air_amigo
* air_kinect
* air_p3dx

Dependency
----------
* geometry_msgs
* nav_msgs
* sensor_msgs
* ROSARIA

Installation
------------
1.  air_amigo

	```
	$ rosdep install air_amigo
	$ rosmake air_amigo
	```
2.  air_kinect

	```
	$ rosdep install air_kinect
	$ rosmake air_kinect
	```
3. 	air_p3dx

	```
	$ rosdep install air_p3dx
	$ rosmake air_p3dx
	```

Known Issues
------------
No issues.

 -- ranjanashish

