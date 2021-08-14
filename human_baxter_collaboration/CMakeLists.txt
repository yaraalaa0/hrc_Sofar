cmake_minimum_required(VERSION 3.0.2)
project(human_baxter_collaboration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  moveit_msgs
  sensor_msgs
  graph_msgs
  controller_manager
  roslaunch
  trajectory_msgs
  moveit_ros_planning_interface
  #moveit_visual_tools
  #rviz_visual_tools
  human_baxter_servers
)
roslaunch_add_file_check(launch)

add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
  graph_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tf_publisher.py
  scripts/server_endpoint_hrc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_niryo_moveit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

include_directories(
${catkin_INCLUDE_DIRS}
)

#add_executable(tf2_listener src/tf2_listener.cpp)
add_executable(task_manager src/task_manager.cpp)
add_executable(task_manager_hand src/task_manager_hand.cpp)
#add_executable(moveit_controller src/moveit_controller.cpp)
#add_executable(robot_joint_state_publisher src/robot_joint_state_publisher.cpp)

#add_dependencies(tf2_listener ${catkin_EXPORTED_TARGETS})
add_dependencies(task_manager ${catkin_EXPORTED_TARGETS})
add_dependencies(task_manager_hand ${catkin_EXPORTED_TARGETS})
#add_dependencies(moveit_controller ${catkin_EXPORTED_TARGETS})
#add_dependencies(robot_joint_state_publisher ${catkin_EXPORTED_TARGETS})

#target_link_libraries(tf2_listener
#${catkin_LIBRARIES}
#)

target_link_libraries(task_manager
${catkin_LIBRARIES}
)

target_link_libraries(task_manager_hand
${catkin_LIBRARIES}
)

#target_link_libraries(moveit_controller
#${catkin_LIBRARIES}
#)

#target_link_libraries(robot_joint_state_publisher
#${catkin_LIBRARIES}
#)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
