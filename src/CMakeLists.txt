cmake_minimum_required(VERSION 3.10)
project(kinect2)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

# ADD Libfreenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io features visualization console filters conversions)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


add_executable(kinect_test main.cpp)
target_link_libraries(kinect_test ${freenect2_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)

install(TARGETS kinect_test DESTINATION ${PROJECT_INSTALL_BIN_DIR})
