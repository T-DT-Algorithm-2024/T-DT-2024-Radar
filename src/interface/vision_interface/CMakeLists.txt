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
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

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
   ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}

  ADD_LINTER_TESTS
 )
ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
