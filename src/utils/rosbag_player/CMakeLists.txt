cmake_minimum_required(VERSION 3.8)
project(rosbag_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准为 C++14，并要求项目必须使用该标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求项目必须使用设置的 C++ 标准，如果编译器不支持该标准，CMake 会报错

# 禁用编译器特定的 C++ 语言扩展
set(CMAKE_CXX_EXTENSIONS OFF)

# 为编译器添加编译选项，-pthread（启用多线程支持），-fexceptions（启用异常支持），-O3（启用最高级别的编译器优化）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

# 添加预处理器宏定义 ROOT_DIR
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#为 C 编译器添加编译选项 -fexceptions，表示启用异常支持
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" ) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(ROSBAG_PLAYER "rosbag_player")
file(GLOB_RECURSE ROSBAG_PLAYER_SRCS "*.cpp")

ament_auto_add_library(${ROSBAG_PLAYER} SHARED
    ${ROSBAG_PLAYER_SRCS}
)

target_link_libraries(${ROSBAG_PLAYER} ${PCL_LIBRARIES})

rclcpp_components_register_node(${ROSBAG_PLAYER}
    PLUGIN "RosbagPlayer"
    EXECUTABLE ${ROSBAG_PLAYER}_node
)

target_link_libraries(rosbag_player_node ${PCL_LIBRARIES})
ament_auto_package()
