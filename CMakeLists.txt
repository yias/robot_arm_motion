cmake_minimum_required(VERSION 2.8.3)
project(robot_arm_motion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mathlib
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  dynamic_reconfigure
  message_generation
#  lpv
  # motion_generators
  utils
#  gaze_tracking
)
message(STATUS "eigen include dir ${EIGEN3_INCLUDE_DIR}")
# find_package(Eigen3 REQUIRED)
# if(NOT Eigen_FOUND)
#   include(cmake/FindEigen3.cmake)
# endif()
# find_package(PythonLibs 2.7)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


generate_dynamic_reconfigure_options(
  cfg/config.cfg
)

add_message_files(
  FILES
  sobs.msg
  obstacle_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES robot_arm_motion
 CATKIN_DEPENDS mathlib roscpp message_runtime std_msgs #lpv
 DEPENDS system_lib #Eigen3
)



###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${PYTHON_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})



add_executable(rds_motion src/robotDSMotion.cpp)
target_link_libraries(rds_motion ${catkin_LIBRARIES})
add_dependencies(rds_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(targetNode src/targetNode.cpp)
target_link_libraries(targetNode ${catkin_LIBRARIES})
add_dependencies(targetNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(markerTargetNode src/markerTarget.cpp src/markerTarget_node.cpp src/sg_filter.cpp)
target_link_libraries(markerTargetNode ${catkin_LIBRARIES})
add_dependencies(markerTargetNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(moveTodesPose src/moveTodesPose.cpp)
target_link_libraries(moveTodesPose ${catkin_LIBRARIES})
add_dependencies(moveTodesPose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)



# set(CORE_SOURCES
# 	src/Obstacle/Obstacle.cpp
#  	src/Obstacle/Ellipsoid.cpp
#  	src/Obstacle/Aggregate.cpp
#  	src/Obstacle/StarShapeHull.cpp
#  	src/State/Pose.cpp
#  	src/State/State.cpp
# 	src/Modulation.cpp
#   src/Agent.cpp
#   src/Environment.cpp
#   src/Utils/MathTools.cpp
#   # src/Utils/Plotting/PlottingTools.cpp
#   src/Utils/ObstacleGeneration/Aggregation.cpp
# )

add_executable(obstacleAvoidance src/doa_node.cpp src/DSObstacleAvoidance.cpp)
# src/Agent.cpp src/Environment.cpp src/Modulation.cpp src/Obstacle/Aggregate.cpp 
# src/Obstacle/Ellipsoid.cpp src/Obstacle/Aggregate.cpp src/Obstacle/Aggregate.cpp )
target_link_libraries(obstacleAvoidance PUBLIC ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
add_dependencies(obstacleAvoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(target_locator src/targetDefiner.cpp)
# target_link_libraries(target_locator ${catkin_LIBRARIES})
# add_dependencies(target_locator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(target_locator ${catkin_EXPORTED_TARGETS})

# add_executable(fake_target_publisher src/fake_target_node.cpp)
# target_link_libraries(fake_target_publisher PUBLIC ${catkin_LIBRARIES})
# add_dependencies(fake_target_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# script to update target position A -> B viceversa
add_executable(fake_target_publisher src/fake_target_node.cpp)
target_link_libraries(fake_target_publisher PUBLIC ${catkin_LIBRARIES})
add_dependencies(fake_target_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fake_gaze_target_publisher src/fake_gaze_target.cpp)
target_link_libraries(fake_gaze_target_publisher PUBLIC ${catkin_LIBRARIES})
add_dependencies(fake_gaze_target_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(obstacle_visualization src/visualization_node.cpp)
target_link_libraries(obstacle_visualization PUBLIC ${catkin_LIBRARIES})
add_dependencies(obstacle_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(visualize_gripper_orient src/visualize_gripper_orient.cpp)
target_link_libraries(visualize_gripper_orient PUBLIC ${catkin_LIBRARIES})
add_dependencies(visualize_gripper_orient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(obstacleAvoidanceClean src/doa_node_clean.cpp src/DSObstacleAvoidance.cpp)
target_link_libraries(obstacleAvoidanceClean PUBLIC ${catkin_LIBRARIES})
add_dependencies(obstacleAvoidanceClean ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(target_selector src/fake_target_only.cpp)
target_link_libraries(target_selector PUBLIC ${catkin_LIBRARIES})
add_dependencies(target_selector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})