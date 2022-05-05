cmake_minimum_required(VERSION 3.5)
project(rct_ros_tools_rclcpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rct_image_tools REQUIRED)
find_package(rct_optimizations REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS features2d core calib3d)

message(WARNING ${OpenCV_VERSION})
message(WARNING ${OpenCV_LIBS})

# TODO: Export these to a different package dedicated to 'ros tools'
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)


###########
## Build ##
###########

# Create a library for loading and saving "data sets": directory based
# archives of poses and images that can be reloaded for testing and
# development purposes.
add_library(${PROJECT_NAME}_data_loader
  src/data_loader/data_set.cpp
  src/data_loader/parameter_loaders.cpp)

target_include_directories(${PROJECT_NAME}_data_loader PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
#  ${rct_image_tools_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_data_loader
 yaml-cpp
 rct::rct_image_tools
 rct::rct_optimizations
 ${OpenCV_LIBS}
# ${rct_image_tools_LIBRARIES}
 ${rclcpp_LIBRARIES}
 ${std_srvs_LIBRARIES}
 ${tf2_ros_LIBRARIES}
 ${tf2_eigen_LIBRARIES}
 ${tf2_geometry_msgs_LIBRARIES}
 ${cv_bridge_LIBRARIES}
 ${image_transport_LIBRARIES}
)

# Executable for collecting data sets via subscribers and triggered with services
# See readme (TODO: Write a readme)
add_executable(${PROJECT_NAME}_cmd src/command_line_cal.cpp)

set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME command_line_data_collection PREFIX "")

target_include_directories(${PROJECT_NAME}_cmd PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
#  ${rct_image_tools_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_cmd
 ${PROJECT_NAME}_data_loader
 rct::rct_image_tools
 rct::rct_optimizations
 ${OpenCV_LIBRARIES}
 ${rclcpp_LIBRARIES}
 ${rct_image_tools_LIBRARIES}
 ${std_srvs_LIBRARIES}
 ${tf2_ros_LIBRARIES}
 ${tf2_eigen_LIBRARIES}
 ${tf2_geometry_msgs_LIBRARIES}
 ${cv_bridge_LIBRARIES}
 ${image_transport_LIBRARIES}
)

add_executable(${PROJECT_NAME}_wrist_cal_node  src/camera_on_wrist_node.cpp)

set_target_properties(${PROJECT_NAME}_wrist_cal_node PROPERTIES OUTPUT_NAME wrist_cal_node PREFIX "")

target_include_directories(${PROJECT_NAME}_wrist_cal_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_wrist_cal_node
 ${PROJECT_NAME}_data_loader
 rct::rct_image_tools
 rct::rct_optimizations
 rct::rct_common
 ament_index_cpp::ament_index_cpp
)

ament_export_include_directories(
    include
    )

ament_export_libraries(
    yaml-cpp
    ${PROJECT_NAME}_data_loader
    )

ament_export_dependencies(
    rclcpp)

ament_package()

#############
## Install ##
#############

set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(TARGETS ${PROJECT_NAME}_data_loader EXPORT ${PROJECT_NAME}_data_loader
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_cmd EXPORT ${PROJECT_NAME}_cmd
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_wrist_cal_node EXPORT ${PROJECT_NAME}_wrist_cal_node
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_cmd
        ${PROJECT_NAME}_wrist_cal_node
        DESTINATION ${ROS_LIB_DESTINATION}/${PROJECT_NAME})
