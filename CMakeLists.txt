cmake_minimum_required(VERSION 2.8.3)
project(fpfh_coarse_alignment)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(PCL REQUIRED)

# include directories
include_directories(
	include
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_test reLocalizer_test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PCL_LIBRARIES})