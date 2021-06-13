cmake_minimum_required(VERSION 3.10)
project(chasing_utils)
set(CMAKE_BUILD_TYPE release)
add_definitions(-std=c++14 )

find_package(catkin REQUIRED
        roscpp  pcl_ros tf visualization_msgs pcl_ros
        nav_msgs
        tf2_sensor_msgs  tf2_ros
        octomap_server)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES chasing_utils
)

add_library(${PROJECT_NAME} src/Library.cpp src/Utils.cpp src/TargetManager.cpp)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(offline_library_generator exe/OfflineLibraryGenerator.cpp   src/Utils.cpp src/Library.cpp )
target_link_libraries(offline_library_generator   ${catkin_LIBRARIES})


add_executable(prediction_node exe/Prediction.cpp  src/Utils.cpp src/Library.cpp src/TargetManager.cpp)
target_link_libraries(prediction_node   ${catkin_LIBRARIES})


