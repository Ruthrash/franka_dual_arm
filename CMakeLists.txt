cmake_minimum_required(VERSION 3.10)
project(dual_arm)

# minimum C++ version
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


find_package(Franka REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Franka_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${Franka_INCLUDE_DIRS})

#build dual arm 
file(GLOB_RECURSE TESTS_SRC  ${CMAKE_CURRENT_LIST_DIR}/src/server.cpp ${CMAKE_CURRENT_LIST_DIR}/src/common.cpp)
add_executable( dual_arm ${TESTS_SRC} )
target_link_libraries(dual_arm  libzmq Franka::Franka ${Franka} Eigen3::Eigen )







