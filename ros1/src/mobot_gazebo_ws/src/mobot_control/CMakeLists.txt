cmake_minimum_required(VERSION 2.8.3)
project(mobot_control)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 

find_package(catkin REQUIRED roscpp rospy moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules std_msgs geometry_msgs sensor_msgs trajectory_msgs control_msgs message_generation tf eigen_conversions)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(FILES Keyboard.msg ArmControl.msg SwingControl.msg GripControl.msg InterfaceJointVel.msg )
generate_messages(DEPENDENCIES std_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS moveit_core moveit_ros_planning_interface interactive_markers std_msgs geometry_msgs sensor_msgs trajectory_msgs control_msgs message_runtime tf)


#############
## includes #
#############
include_directories(SYSTEM)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


###########
## Build ##
###########
add_executable(mobot_gazebo_preprocess_key src/mobot_gazebo_preprocess_key.cpp)
add_dependencies(mobot_gazebo_preprocess_key mobot_control_generate_messages_cpp)
target_link_libraries(mobot_gazebo_preprocess_key ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mobot_gazebo_preprocess_key DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(joint_states_combined src/joint_states_combined.cpp)
add_dependencies(joint_states_combined mobot_control_generate_messages_cpp)
target_link_libraries(joint_states_combined ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joint_states_combined DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mobot_gazebo_manipulator src/mobot_gazebo_manipulator.cpp)
add_dependencies(mobot_gazebo_manipulator mobot_control_generate_messages_cpp)
target_link_libraries(mobot_gazebo_manipulator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mobot_gazebo_manipulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mobot_gazebo_interface src/mobot_gazebo_interface.cpp)
add_dependencies(mobot_gazebo_interface mobot_control_generate_messages_cpp)
target_link_libraries(mobot_gazebo_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mobot_gazebo_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(vehicle_gazebo_coordinator src/vehicle_gazebo_coordinator.cpp)
add_dependencies(vehicle_gazebo_coordinator mobot_control_generate_messages_cpp)
target_link_libraries(vehicle_gazebo_coordinator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(vehicle_gazebo_interface src/vehicle_gazebo_interface.cpp)
add_dependencies(vehicle_gazebo_interface mobot_control_generate_messages_cpp)
target_link_libraries(vehicle_gazebo_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS vehicle_gazebo_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/gazebo_keyboard.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


