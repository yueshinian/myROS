cmake_minimum_required(VERSION 2.8.3)
project(mobot6_interface)

find_package(catkin REQUIRED qt_build roscpp rospy moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules std_msgs geometry_msgs sensor_msgs trajectory_msgs control_msgs message_generation tf eigen_conversions)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(FILES Keyboard.msg ArmControl.msg SwingControl.msg GripControl.msg InterfaceJointVel.msg command_list.msg heart_reconnection.msg KeyboardNuclear.msg NuclearControl.msg Intensity.msg PointIntensity.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS moveit_core moveit_ros_planning_interface interactive_markers std_msgs geometry_msgs sensor_msgs trajectory_msgs control_msgs message_runtime tf)

rosbuild_prepare_qt4(QtCore QtGui)

#############
## includes #
#############
include_directories(SYSTEM)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu/)
LINK_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/opt/qt57/include/)

###########
## Build ##
###########
add_executable(joint6_states_blend src/joint6_states_blend.cpp)
add_dependencies(joint6_states_blend mobot6_interface_generate_messages_cpp)
target_link_libraries(joint6_states_blend ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joint6_states_blend DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mobot6_preprocess_key src/mobot6_preprocess_key.cpp)
add_dependencies(mobot6_preprocess_key mobot6_interface_generate_messages_cpp)
target_link_libraries(mobot6_preprocess_key ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mobot6_preprocess_key DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(nuclear_preprocess_key src/nuclear_preprocess_key.cpp)
add_dependencies(nuclear_preprocess_key mobot6_interface_generate_messages_cpp)
target_link_libraries(nuclear_preprocess_key ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS nuclear_preprocess_key DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mobot6_preprocess_qt src/mobot6_preprocess_qt.cpp)
add_dependencies(mobot6_preprocess_qt mobot6_interface_generate_messages_cpp)
target_link_libraries(mobot6_preprocess_qt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mobot6_preprocess_qt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mobot6_manipulator src/mobot6_manipulator.cpp)
add_dependencies(mobot6_manipulator mobot6_interface_generate_messages_cpp)
target_link_libraries(mobot6_manipulator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mobot6_manipulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mobot6_interface src/mobot6_interface.cpp)
add_dependencies(mobot6_interface mobot6_interface_generate_messages_cpp)
target_link_libraries(mobot6_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} modbus)
install(TARGETS mobot6_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(vehicle61_coordinator src/vehicle61_coordinator.cpp)
add_dependencies(vehicle61_coordinator mobot6_interface_generate_messages_cpp)
target_link_libraries(vehicle61_coordinator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS vehicle61_coordinator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(vehicle61_interface src/vehicle61_interface.cpp)
add_dependencies(vehicle61_interface mobot6_interface_generate_messages_cpp)
target_link_libraries(vehicle61_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} modbus)
install(TARGETS vehicle61_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(node6_check src/node6_check)
add_dependencies(node6_check mobot6_interface_generate_messages_cpp)
target_link_libraries(node6_check ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS node6_check DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(twins socket_src/main.cpp socket_src/heartbeat_recv_socket.cpp socket_src/heartbeat_send_socket.cpp socket_src/Revmsgs_Socket.cpp)
add_dependencies(twins mobot6_interface_generate_messages_cpp)
target_link_libraries(twins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)

add_executable(socket_send socket_send/main.cpp socket_send/Sendmsgs_Socket.cpp)
add_dependencies(socket_send mobot6_interface_generate_messages_cpp)
target_link_libraries(socket_send ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)

add_executable(fake_joint_states_vehicle src/fake_joint_states_vehicle.cpp)
add_dependencies(fake_joint_states_vehicle mobot6_interface_generate_messages_cpp)
target_link_libraries(fake_joint_states_vehicle ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS fake_joint_states_vehicle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/mobot6_keyboard.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
