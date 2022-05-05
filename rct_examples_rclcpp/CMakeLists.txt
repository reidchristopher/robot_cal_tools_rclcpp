cmake_minimum_required(VERSION 3.5)
project(rct_examples_rclcpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rct_optimizations REQUIRED)
find_package(rct_image_tools REQUIRED)
find_package(rct_common REQUIRED)
find_package(rct_ros_tools_rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
if(NOT TARGET Eigen3::Eigen)
    find_package(Threads REQUIRED)
    add_library(Eigen3::Eigen IMPORTED INTERFACE)
    set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
    set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

add_executable(${PROJECT_NAME}_wrist_example src/camera_on_wrist.cpp)

set_target_properties(${PROJECT_NAME}_wrist_example PROPERTIES OUTPUT_NAME example_camera_on_wrist PREFIX "")

target_include_directories(${PROJECT_NAME}_wrist_example PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rct_ros_tools_rclcpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_wrist_example
 rct::rct_image_tools
 rct::rct_optimizations
 rct::rct_common
 ament_index_cpp::ament_index_cpp
 ${rct_ros_tools_rclcpp_LIBRARIES}
)


ament_target_dependencies(${PROJECT_NAME}_wrist_example
  rclcpp
  rct_optimizations
  rct_image_tools
  rct_common
  rct_ros_tools_rclcpp)

#ament_export_include_directories(
#    include
#    )

#ament_export_libraries(
#    ${PROJECT_NAME}_wrist_example
#    )

#ament_export_dependencies(
#    rclcpp)

install(TARGETS ${PROJECT_NAME}_wrist_example DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/)


ament_package()

#install(TARGETS ${PROJECT_NAME}_wrist_example EXPORT ${PROJECT_NAME}_wrist_example
#  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
#  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
#  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
#)
