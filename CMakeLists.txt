cmake_minimum_required(VERSION 3.5)
project(robot_serial)

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


# find dependencies and libraries
find_package(ament_cmake REQUIRED)

set(AMENT_DEPENDENCIES
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
)

foreach (AMENT_DEPENDENCY ${AMENT_DEPENDENCIES})
    find_package(${AMENT_DEPENDENCY} REQUIRED)
endforeach ()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Aim.msg"
        "msg/Armor.msg"
        "msg/Chassis.msg"
        "msg/DialSwitch.msg"
        "msg/Gimbal.msg"
        "msg/Mode.msg"
        "msg/SpinningControl.msg"
        DEPENDENCIES std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(thirdparty/serialib)

set(LIBRARIES
        ${OpenCV_LIBS}
        serialib
)

include_directories(include thirdparty/serialib ${EIGEN3_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_node
        src/robot_serial_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node ${LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS serialib
        DESTINATION lib)

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