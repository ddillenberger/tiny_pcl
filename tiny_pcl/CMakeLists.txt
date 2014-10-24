cmake_minimum_required(VERSION 2.8.3)
project(tiny_pcl)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  message_runtime
)
find_library(LIBZ_LIBRARIES NAMES z)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CPointCloud2.msg
  CPointField.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tiny_pcl
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS libz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
	src/tinypcl.cpp
	src/bitdatamem.cpp
	src/pclcontainer.cpp
        src/stopwatch.cpp
)



set(HEADER_FILES
	src/tinypcl.h
	src/bitdatamem.h
	src/pclcontainer.h
)

## Declare a cpp library
add_library(tiny_pcl
	${SOURCE_FILES}
)

## Declare a cpp executable
# add_executable(tiny_pcl_node src/tiny_pcl_node.cpp)

add_executable(pointcloud_transport
    src/pointcloud_transport_node.cpp
    src/stringutil.cpp
    src/readwriteutil.cpp
)
add_executable(test_pcl_transport
    src/test_pcl_transport.cpp
)
add_executable(pointcloud_repeater
    src/pointcloud_repeater.cpp
    src/stringutil.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pointcloud_transport tiny_pcl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pointcloud_transport
  ${catkin_LIBRARIES}
  ${LIBZ_LIBRARIES}
  tiny_pcl
)
target_link_libraries(test_pcl_transport
  ${catkin_LIBRARIES}
  ${LIBZ_LIBRARIES}
  tiny_pcl
)
target_link_libraries(pointcloud_repeater
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tiny_pcl tiny_pcl_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tiny_pcl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
