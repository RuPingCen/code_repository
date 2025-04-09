nmea_navsat_driver
===============

从蓝鲸机器人GPS读取包中拷贝过来，修复了 “ImportError: No module named gps_common.msg” 问题

sudo apt-get install ros-melodic-gps-common

如果使用的是Python2的环境需要把 nmea_navsat_driver/scripts 目录下面的四个文件第一行添加 “ #! /usr/bin/python2.7 ”

#蓝鲸机器人GPS读取包

https://github.com/BluewhaleRobot/nmea_navsat_driver/tree/master

增加北斗卫星支持，增加发布 [gps_common/GPSFix](http://docs.ros.org/api/gps_common/html/msg/GPSFix.html) 话题，方便用户了解卫星连接状态

Add support for Beidou, add topic `extend_fix`, make it easier for user to get info about GPS status.

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver
