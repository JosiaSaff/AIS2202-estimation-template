#find_package(Eigen3 CONFIG REQUIRED)
#find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

#add_executable(parameter_estimation main.cpp)

#target_link_libraries(parameter_estimation PUBLIC Eigen3::Eigen)
#target_include_directories(parameter_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})

find_package(Eigen3 CONFIG REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_executable(stateEstimation main.cpp)

target_link_libraries(stateEstimation PUBLIC Eigen3::Eigen estimation)
target_include_directories(stateEstimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})