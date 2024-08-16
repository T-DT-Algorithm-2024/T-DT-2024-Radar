cmake_minimum_required(VERSION 3.8)
project(debug_map)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

ament_auto_add_executable(debug_map debug_map.cpp)

ament_auto_package ()