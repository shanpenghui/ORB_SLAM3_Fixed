cmake_minimum_required(VERSION 3.5)
project(opti_track)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/TrackData.msg
        msg/Marker.msg
        msg/RigidBody.msg
        DEPENDENCIES std_msgs geometry_msgs
        )

link_directories(lib)

add_executable(opti_track_client src/OptiTrackClient.cpp)
target_include_directories(opti_track_client PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(opti_track_client libNatNet.so)
ament_target_dependencies(opti_track_client
        rclcpp
        std_msgs)


rosidl_target_interfaces(opti_track_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY lib/
        DESTINATION lib/)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(TARGETS opti_track_client
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
