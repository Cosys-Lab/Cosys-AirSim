cmake_minimum_required(VERSION 3.12.0)
project(AirSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory("rpclib_wrapper")
add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("AirLibUnitTests")
add_subdirectory("HelloDrone")
add_subdirectory("HelloSpawnedDrones")
add_subdirectory("HelloCar")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")


