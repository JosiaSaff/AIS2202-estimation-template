cmake_minimum_required(VERSION 3.27)
project(AIS2202-estimation-template)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(estimation)
add_subdirectory(parameter_estimation)
add_subdirectory(state_estimation)

add_dependencies(state_estimation estimation)
