cmake_minimum_required(VERSION 3.5)
project(orb_slam3)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(opti_track REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV 4.5.0)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.0 not found.")
    endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/TrackData.msg
        msg/Marker.msg
        msg/RigidBody.msg
        DEPENDENCIES std_msgs geometry_msgs
        )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../../../../include
        ${PROJECT_SOURCE_DIR}/../../../../../include/CameraModels
        ${PROJECT_SOURCE_DIR}/../../../../..
        /usr/local/include/opencv4
)

set(LIBS
        ${PROJECT_SOURCE_DIR}/../../../../../Thirdparty/DBoW2/build/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../../../../Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/../../../../../lib/libORB_SLAM3.so
        -lboost_system
        ${OpenCV_LIBS}
        )

add_executable(mono_inertial src/ros_mono_inertial.cpp)
ament_target_dependencies(mono_inertial rclcpp std_msgs sensor_msgs realsense2_camera_msgs realsense2_description realsense2_camera Eigen3 Pangolin opti_track)

target_link_libraries(mono_inertial ${LIBS})
target_include_directories(mono_inertial PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(mono_inertial ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        mono_inertial
        DESTINATION lib/${PROJECT_NAME})

ament_package()
