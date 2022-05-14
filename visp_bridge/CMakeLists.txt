cmake_minimum_required(VERSION 3.5)
project(visp_bridge)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

#include_directories(SYSTEM
#  ${VISP_INCLUDE_DIRS}
#  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
#)

include_directories(include)

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

## Declare a cpp library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/3dpose.h
  include/${PROJECT_NAME}/camera.h
  include/${PROJECT_NAME}/image.h
  include/${PROJECT_NAME}/vpQuaternionVector.h
  src/compat/vpQuaternionVector.cpp
  src/conversions/3dpose.cpp
  src/conversions/camera.cpp
  src/conversions/image.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components sensor_msgs geometry_msgs )
target_link_libraries(${PROJECT_NAME} ${VISP_LIBRARIES})

add_executable(${PROJECT_NAME}_convert_camera_parameters src/conversions/convert_cam_param_file.cpp)
target_include_directories(${PROJECT_NAME}_convert_camera_parameters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_convert_camera_parameters rclcpp rclcpp_components sensor_msgs geometry_msgs camera_calibration_parsers)
target_link_libraries(${PROJECT_NAME}_convert_camera_parameters ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_convert_camera_parameters 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(visp_bridge)

ament_package()
