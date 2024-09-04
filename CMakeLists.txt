cmake_minimum_required(VERSION 3.5)
project(orbslam3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV REQUIRED)
<<<<<<< HEAD
find_package(builtin_interfaces REQUIRED)
=======
<<<<<<< HEAD
find_package(builtin_interfaces REQUIRED)
=======
>>>>>>> 37975756a0a49fb502eea3f17cc67294dc7b630f
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24

# Include directories
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}
)

# Add mono executable
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

# Specify dependencies for mono
ament_target_dependencies(mono 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  ORB_SLAM3 
  Pangolin 
  OpenCV
<<<<<<< HEAD
  builtin_interfaces
=======
<<<<<<< HEAD
  builtin_interfaces
=======
>>>>>>> 37975756a0a49fb502eea3f17cc67294dc7b630f
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24
)

# Link libraries for mono
target_link_libraries(mono
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${ORB_SLAM3_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
<<<<<<< HEAD
  ${builtin_interfaces_LIBRARIES}
=======
<<<<<<< HEAD
  ${builtin_interfaces_LIBRARIES}
=======
>>>>>>> 37975756a0a49fb502eea3f17cc67294dc7b630f
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24
)

# Install mono executable
install(TARGETS mono
  DESTINATION lib/${PROJECT_NAME}
)

<<<<<<< HEAD
ament_package()
=======
<<<<<<< HEAD
ament_package()
=======
ament_package()
>>>>>>> 37975756a0a49fb502eea3f17cc67294dc7b630f
>>>>>>> 47e7faea95fe71c96f25a95a31caeba224c7be24
