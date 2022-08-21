cmake_minimum_required(VERSION 3.5)
project(mcu_ota)
SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color=always)
endif ()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cti_chassis_msgs REQUIRED)
find_package(ctilog REQUIRED)
find_package(cti_msgs REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/[a-zA-Z]*.c" "src/[a-zA-Z]*.cc" "src/[a-zA-Z]*.cpp" "include/[a-zA-Z]*.cpp" "include/[a-zA-Z]*.c" "include/[a-zA-Z]*.cc")

add_executable(${PROJECT_NAME}_node ${ALL_SRCS})
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        std_msgs
        geometry_msgs
        cti_chassis_msgs
        cti_msgs
        ctilog
)

target_link_libraries(${PROJECT_NAME}_node
        pthread
        ${catkin_LIBRARIES}
)
#install
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION share/${PROJECT_NAME}
)
ament_package()