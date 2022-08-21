cmake_minimum_required(VERSION 2.8.3)
project(cti_fpga_serial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  message_generation
  sensor_msgs
  std_msgs
  tf
  cti_msgs
  ctilog
  cti_monitor
)
find_package(Eigen3)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})

##json
find_package(PkgConfig REQUIRED) 
pkg_search_module(jsoncpp REQUIRED jsoncpp) 

add_message_files(
    FILES
    cmdanswer.msg
    updateinfo.msg
    navigationlog.msg
    serialstatus.msg
    firmwareinfo.msg
    vehiclestate.msg
    dustbinidstate.msg
    rangeRawData.msg
    cleanFuncTestState.msg
    LoraIdSetState.msg
    ultModeSetState.msg
    fwVerCheckState.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cti_fpga_serial
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs serial std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB_RECURSE ALL_SRCS "src/[a-zA-Z]*.c" "src/[a-zA-Z]*.cc" "src/[a-zA-Z]*.cpp" "include/[a-zA-Z]*.cpp" "include/[a-zA-Z]*.c" "include/[a-zA-Z]*.cc")
file(GLOB_RECURSE ALL_HEDS "include/[a-zA-Z]*.hpp" "include/[a-zA-Z]*.h")

add_executable(${PROJECT_NAME}_node ${ALL_SRCS}  ${ALL_HEDS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} pthread   ${jsoncpp_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

