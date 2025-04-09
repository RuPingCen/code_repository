# Install script for directory: /home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/man

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man1" TYPE FILE FILES
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/CartConvert.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/ConicProj.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/GeodesicProj.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/GeoConvert.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/GeodSolve.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/GeoidEval.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/Gravity.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/IntersectTool.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/MagneticField.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/Planimeter.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/RhumbSolve.1"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/TransverseMercatorProj.1"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man8" TYPE FILE FILES
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/geographiclib-get-geoids.8"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/geographiclib-get-gravity.8"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/man/geographiclib-get-magnetic.8"
    )
endif()

