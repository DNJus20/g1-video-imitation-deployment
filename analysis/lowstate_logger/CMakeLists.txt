cmake_minimum_required(VERSION 3.10)
project(lowstate_logger)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

include_directories(
  /usr/local/include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

add_executable(lowstate_logger lowstate_logger.cpp)
target_link_libraries(lowstate_logger
  unitree_sdk2
  ddsc
  ddscxx
  pthread
  rt
)

add_executable(lowcmd_pubs lowcmd_pubs.cpp)
target_link_libraries(lowcmd_pubs unitree_sdk2 ddsc ddscxx pthread rt)
