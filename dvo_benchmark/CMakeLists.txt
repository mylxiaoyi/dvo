cmake_minimum_required(VERSION 2.4.6)
project(dvo_benchmark)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dvo_core
    dvo_ros    
)
include_directories(${catkin_INCLUDE_DIRS} include)

# Eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})

catkin_package()

add_executable(benchmark
     src/benchmark.cpp
)

target_link_libraries(benchmark
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)
