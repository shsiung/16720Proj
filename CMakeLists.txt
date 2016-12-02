cmake_minimum_required(VERSION 2.8.3)
project(lsd_alignment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  sensor_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
)

INCLUDE_DIRECTORIES("/usr/include/eigen3")

# Find GTSAM
find_package(GTSAM REQUIRED)
find_package(OpenCV)
find_package(PCL 1.7 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIRS})
include_directories(  ${PCL_INCLUDE_DIRS} )
include_directories(includes)
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM INCS: " ${GTSAM_EIGEN_INCLUDE_PREFIX})
message("GTSAM LIBS: " ${GTSAM_LIBRARY_DIRS})

# BOOST
find_package(Boost COMPONENTS filesystem system  REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# TBB
find_package(cmake_modules REQUIRED)
find_package(TBB COMPONENTS tbbmalloc)

###################################
## catkin specific configuration ##
###################################
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lsd_alignment_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS Eigen
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Declare a C++ executable
add_executable(lsd_alignment src/main.cpp src/lsd.cpp src/camera.cpp)
target_link_libraries(lsd_alignment ${catkin_LIBRARIES} ${GTSAM_LIBRARIES} ${TBB_LIBRARIES} )
target_link_libraries(lsd_alignment ${Boost_LIBRARIES})
target_link_libraries(lsd_alignment ${Boost_SYSTEM_LIBRARY})
target_link_libraries(lsd_alignment ${OpenCV_LIBRARIES})
#add_dependencies(lsd_alignment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
