cmake_minimum_required(VERSION 3.8)
project(dynamic_cloud)
set(CMAKE_BUILD_TYPE "RELEASE")
find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

include_directories(include)

set(DYNAMIC_CLOUD "dynamic_cloud")
file(GLOB_RECURSE DYNAMIC_CLOUD_SRCS "src/*.cpp")

ament_auto_add_library(${DYNAMIC_CLOUD} SHARED
    ${DYNAMIC_CLOUD_SRCS}
)

target_link_libraries(${DYNAMIC_CLOUD} ${PCL_LIBRARIES})

rclcpp_components_register_node(${DYNAMIC_CLOUD}
    PLUGIN "tdt_radar::DynamicCloud"
    EXECUTABLE ${DYNAMIC_CLOUD}_node
)

target_link_libraries(dynamic_cloud_node ${PCL_LIBRARIES})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package ()