cmake_minimum_required(VERSION 2.8)
project(TEST)

find_package(Eigen3 3.1.0 REQUIRED)

add_executable(test_quat test/test_quat_main.cc)

target_link_libraries(test_quat ${EIGEN3_LIBS})

add_executable(test_quat_2_ test/test_quat_main_2.cc)

target_link_libraries(test_quat_2_ ${EIGEN3_LIBS})