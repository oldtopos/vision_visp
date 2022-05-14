cmake_minimum_required(VERSION 3.5)
project(visp_camera_calibration)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(VISP REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(rcl REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

#include_directories(SYSTEM
#  ${BULLET_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
#  ${VISP_INCLUDE_DIRS}
#  )


include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibPoint.msg"
  "msg/CalibPointArray.msg"
  "msg/ImageAndPoints.msg"
  "msg/ImagePoint.msg"
  "srv/Calibrate.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_common src/names.cpp src/names.h)
ament_target_dependencies(${PROJECT_NAME}_common rclcpp sensor_msgs geometry_msgs )

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_calibrator
  src/calibrator.cpp
  src/calibrator.h
  src/calibrator_main.cpp)
target_link_libraries(${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_common
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_calibrator rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge )

add_executable(${PROJECT_NAME}_image_processing
  src/image_processing.cpp
  src/image_processing.h
  src/image_processing_main.cpp)
target_link_libraries(${PROJECT_NAME}_image_processing
  ${PROJECT_NAME}_common
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}"
  )
ament_target_dependencies(${PROJECT_NAME}_image_processing rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge camera_calibration_parsers)

add_executable(${PROJECT_NAME}_camera
  src/camera.cpp
  src/camera.h
  src/camera_main.cpp)
target_link_libraries(${PROJECT_NAME}_camera
  ${PROJECT_NAME}_common
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_camera rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge camera_calibration_parsers)


install(
  TARGETS
    ${PROJECT_NAME}_calibrator
    ${PROJECT_NAME}_camera
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_image_processing
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
