cmake_minimum_required(VERSION 3.10)
project(kinect_test)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

set(LIB_PATH "/home/sa9472/freenect2/")
find_package(freenect2 PATHS $LIB_PATH)
include_directories(${freenect2_INCLUDE_DIR})

find_package(PCL 1.8 REQUIRED COMPONENTS common io features visualization console filters conversions)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(kinect_test main.cpp)

target_link_libraries(kinect_test ${freenect2_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)
