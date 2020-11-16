cmake_minimum_required(VERSION 3.5)
project(iiwa_examples)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(trial src/command_robot.cpp)
add_dependencies(trial iiwa_msgs_generate_messages_cpp)
target_link_libraries(trial ${catkin_LIBRARIES})

add_executable(main_pose src/trial1.cpp)
add_dependencies(main_pose iiwa_msgs_generate_messages_cpp)
target_link_libraries(main_pose ${catkin_LIBRARIES})
