cmake_minimum_required(VERSION 3.5)
project(lidar_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


find_package(PCL 1.3 REQUIRED) 
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN_LIBRARIES_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/ICP_3D)



add_executable(test_icp test_icp.cc icp_svd.cpp)

target_link_libraries(test_icp
                        ${EIGEN_LIBRARIES} 
                        ${PCL_LIBRARIES} 
                        fmt
                        )
