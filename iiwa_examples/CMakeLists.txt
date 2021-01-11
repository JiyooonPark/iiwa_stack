cmake_minimum_required(VERSION 3.5)
project(iiwa_examples)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(updown src/updown.cpp)
add_dependencies(updown iiwa_msgs_generate_messages_cpp)
target_link_libraries(updown ${catkin_LIBRARIES})

add_executable(main_pose src/trial1.cpp)
add_dependencies(main_pose iiwa_msgs_generate_messages_cpp)
target_link_libraries(main_pose ${catkin_LIBRARIES})

add_executable(drawing_plane src/DrawingPlane.cpp)
add_dependencies(drawing_plane iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_plane ${catkin_LIBRARIES})

add_executable(dynamic_lines src/Line_rviz_org.cpp)
add_dependencies(dynamic_lines iiwa_msgs_generate_messages_cpp)
target_link_libraries(dynamic_lines ${catkin_LIBRARIES})

add_executable(drawing_moveit src/drawing_moveit.cpp)
add_dependencies(drawing_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_moveit ${catkin_LIBRARIES})

add_executable(updown_moveit src/updown_moveit.cpp)
add_dependencies(updown_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(updown_moveit ${catkin_LIBRARIES})
