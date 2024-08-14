cmake_minimum_required(VERSION 3.8)
project(cluster)
set(CMAKE_BUILD_TYPE "RELEASE")
find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

include_directories(include)

set(CLUSTER_NODE "cluster")
file(GLOB_RECURSE CLUSTER_NODE_SRCS "src/*.cpp")

ament_auto_add_library(${CLUSTER_NODE} SHARED
    ${CLUSTER_NODE_SRCS}
)

target_link_libraries(${CLUSTER_NODE} ${PCL_LIBRARIES})

rclcpp_components_register_node(${CLUSTER_NODE}
    PLUGIN "tdt_radar::Cluster"
    EXECUTABLE ${CLUSTER_NODE}_node
)
ament_auto_package ()