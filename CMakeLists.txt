cmake_minimum_required(VERSION 3.29)
project(AIS2202-assignments-template)

set(CMAKE_CXX_STANDARD 20)

file(COPY ${CMAKE_SOURCE_DIR}/datasets DESTINATION ${CMAKE_BINARY_DIR})

add_subdirectory(estimation)
add_subdirectory(parameter_estimation)
add_subdirectory(state_estimation)

add_dependencies(state_estimation estimation)