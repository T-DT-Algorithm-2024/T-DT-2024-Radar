cmake_minimum_required(VERSION 3.8)
project(vision_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)

# find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)



file(GLOB_RECURSE 
    MSG_FILES 
    RELATIVE 
    
    ${CMAKE_CURRENT_SOURCE_DIR}
    msg/*.msg
)


rosidl_generate_interfaces(${PROJECT_NAME}

  ${MSG_FILES}
  ${ACTION_FILES}
  ${SRV_FILES}

  
   DEPENDENCIES 
   geometry_msgs
   builtin_interfaces
   std_msgs
   sensor_msgs

  ADD_LINTER_TESTS
 )
ament_export_dependencies(rosidl_default_runtime)


ament_package()
