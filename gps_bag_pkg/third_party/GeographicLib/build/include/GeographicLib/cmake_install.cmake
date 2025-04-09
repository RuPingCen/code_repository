# Install script for directory: /home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Accumulator.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/AlbersEqualArea.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/AuxAngle.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/AuxLatitude.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/AzimuthalEquidistant.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/CassiniSoldner.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/CircularEngine.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Constants.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/DAuxLatitude.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/DMS.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/DST.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Ellipsoid.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/EllipticFunction.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GARS.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GeoCoords.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Geocentric.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Geodesic.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GeodesicExact.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GeodesicLine.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GeodesicLineExact.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Geohash.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Geoid.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Georef.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Gnomonic.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GravityCircle.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/GravityModel.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Intersect.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/LambertConformalConic.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/LocalCartesian.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/MGRS.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/MagneticCircle.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/MagneticModel.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Math.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/NearestNeighbor.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/NormalGravity.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/OSGB.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/PolarStereographic.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/PolygonArea.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Rhumb.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/SphericalEngine.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/SphericalHarmonic.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/SphericalHarmonic1.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/SphericalHarmonic2.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/TransverseMercator.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/TransverseMercatorExact.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/UTMUPS.hpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/include/GeographicLib/Utility.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/build/include/GeographicLib/Config.h")
endif()

