cmake_minimum_required(VERSION 3.5)
project(visp_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(VISP REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KltPoint.msg"
  "msg/KltPoints.msg"
  "msg/KltSettings.msg"
  "msg/MovingEdgeSettings.msg"
  "msg/MovingEdgeSite.msg"
  "msg/MovingEdgeSites.msg"
  "msg/TrackerSettings.msg"
  "srv/Init.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


#############
# Libraries #
#############

# Library gathering libvisp_tracker used by all nodes.
add_library(${PROJECT_NAME}_common
  src/lib${PROJECT_NAME}/callbacks.cpp
  src/lib${PROJECT_NAME}/callbacks.hh
  src/lib${PROJECT_NAME}/conversion.cpp
  src/lib${PROJECT_NAME}/conversion.hh
  src/lib${PROJECT_NAME}/file.cpp
  src/lib${PROJECT_NAME}/file.hh
  src/lib${PROJECT_NAME}/names.cpp
  src/lib${PROJECT_NAME}/names.hh
  src/lib${PROJECT_NAME}/parameter_info.cpp
  src/lib${PROJECT_NAME}/parameter_info.hpp
#  src/lib${PROJECT_NAME}/tracker-client.cpp
#  src/lib${PROJECT_NAME}/tracker-client.hh
#  src/lib${PROJECT_NAME}/tracker-viewer.cpp
#  src/lib${PROJECT_NAME}/tracker-viewer.hh
  src/lib${PROJECT_NAME}/tracker.cpp
  src/lib${PROJECT_NAME}/tracker.hh
)

target_link_libraries(${PROJECT_NAME}_common
  ${Boost_LIBRARIES}
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}"
  )
ament_target_dependencies(${PROJECT_NAME}_common rclcpp rclcpp_components std_msgs sensor_msgs geometry_msgs image_proc image_transport resource_retriever tf2 tf2_ros)

ament_export_include_directories(include)

ament_package()
