cmake_minimum_required(VERSION 3.5)
project(cti_chassis_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color=always)
endif()





# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ChassisErrorCode.msg"
  "msg/CmdAnswer.msg"
  "msg/DustbinidState.msg"
  "msg/FirmWareInfo.msg"
  "msg/NavigationLog.msg"
  "msg/RangeRawData.msg"
  "msg/SerialStatus.msg"
  "msg/UpdateInfo.msg"
  "msg/VehicleState.msg"
  "msg/CommData.msg"
  "msg/UltV30Datas.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  std_msgs
  )

ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
