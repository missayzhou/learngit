cmake_minimum_required(VERSION 2.8.3)
project(IK_fast_solver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  serial
  std_msgs
  message_generation
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  grasp.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES IK_fast_solver
  CATKIN_DEPENDS message_runtime roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

#add_executable(IK_fast_solver_node src/IK_fast_solver.cpp)
link_directories(${OpenCV_LIB_DIR})
#add_executable(zhou_x_arm src/zhou_x_arm.cpp)
#add_executable(ik_moveit_pvt src/ik_moveit_pvt.cpp)
#target_link_libraries(ik_moveit_pvt ${catkin_LIBRARIES} ${Boost_LIBRARIES}   ${OpenCV_LIBS})
#target_link_libraries(IK_fast_solver_node ${catkin_LIBRARIES}  ${OpenCV_LIBS})

file (GLOB SRCS
  "./src/*.cpp"
  )

file (GLOB HEADERS
  "./include/x_arm_hand_grasp_planning/*.h"
  )

add_executable(${PROJECT_NAME}_test_random ./src/test_random.cpp ./src/addObject.cpp ./src/collisionCheck.cpp ./src/line.cpp ${HEADERS})
#add_executable(${PROJECT_NAME}_pose_scan_patch ./src/poseScanPatch.cpp ./src/addObject.cpp ./src/collisionCheck.cpp ${HEADERS})


#add_executable(${PROJECT_NAME}_pose_scan_forLoop ./src/poseScanForLoop.cpp ./src/addObject.cpp ./src/collisionCheck.cpp ${HEADERS})

target_link_libraries(${PROJECT_NAME}_test_random ${catkin_LIBRARIES} ${Boost_LIBRARIES}   ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_pose_scan_patch ${catkin_LIBRARIES} ${Boost_LIBRARIES}   ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_pose_scan_forLoop ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OpenCV_LIBS})

