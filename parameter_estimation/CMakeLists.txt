#find_package(Eigen3 CONFIG REQUIRED)
#find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

#add_executable(parameter_estimation main.cpp)

#target_link_libraries(parameter_estimation PUBLIC Eigen3::Eigen)
#target_include_directories(parameter_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})

# set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/eigen")
# include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_executable(parameter_estimation main.cpp)

target_link_libraries(parameter_estimation PRIVATE Eigen3::Eigen)
target_include_directories(parameter_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})