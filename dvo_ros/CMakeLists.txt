cmake_minimum_required(VERSION 2.4.6)
project(dvo_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure    
    cv_bridge
    dvo_core
    tf_conversions
    interactive_markers
    pcl_ros
    eigen_conversions
)

generate_dynamic_reconfigure_options(cfg/dvo.cfg)

catkin_package(
    INCLUDE_DIRS include cfg/cpp
    LIBRARIES ${PROJECT_NAME}
    )

include_directories(${catkin_INCLUDE_DIRS} include cfg/cpp)
# Eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
    src/camera_base.cpp
    src/camera_dense_tracking.cpp
    
    src/visualization/ros_camera_trajectory_visualizer.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}    
)

add_library(camera_tracker_nodelet
    src/camera_tracker_nodelet.cpp
)

target_link_libraries(camera_tracker_nodelet
    ${PROJECT_NAME}
)

add_executable(camera_tracker
     src/camera_tracker_node.cpp
)

target_link_libraries(camera_tracker
    ${PROJECT_NAME}
)
