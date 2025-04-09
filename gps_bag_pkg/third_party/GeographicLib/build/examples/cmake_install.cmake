# Install script for directory: /home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/GeographicLib-dev" TYPE FILE FILES
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/CMakeLists.txt"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Accumulator.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-AlbersEqualArea.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-AuxAngle.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-AuxLatitude.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-AzimuthalEquidistant.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-CassiniSoldner.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-CircularEngine.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Constants.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-DMS.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-DST.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Ellipsoid.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-EllipticFunction.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GARS.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GeoCoords.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Geocentric.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Geodesic.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Geodesic-small.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GeodesicExact.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GeodesicLine.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GeodesicLineExact.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GeographicErr.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Geohash.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Geoid.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Georef.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Gnomonic.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GravityCircle.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-GravityModel.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Intersect.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-LambertConformalConic.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-LocalCartesian.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-MGRS.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-MagneticCircle.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-MagneticModel.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Math.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-NearestNeighbor.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-NormalGravity.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-OSGB.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-PolarStereographic.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-PolygonArea.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Rhumb.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-RhumbLine.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-SphericalEngine.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-SphericalHarmonic.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-SphericalHarmonic1.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-SphericalHarmonic2.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-TransverseMercator.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-TransverseMercatorExact.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-UTMUPS.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/example-Utility.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/GeoidToGTX.cpp"
    "/home/crp/catkin_mutil_sensor_fusion/src/gps_bag_pkg/third_party/GeographicLib/examples/make-egmcof.cpp"
    )
endif()

