cmake_minimum_required(VERSION 3.10)
project(g1_launcher)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

include_directories(
  /usr/local/include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

add_executable(g1_launcher g1_launcher.cpp)
target_link_libraries(g1_launcher
  unitree_sdk2
  ddsc
  ddscxx
  pthread
  rt
)
