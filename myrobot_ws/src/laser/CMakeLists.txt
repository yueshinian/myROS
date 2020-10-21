cmake_minimum_required(VERSION 2.8.3)
project(laser)

#rosbuild_prepare_qt4(QtCore QtGui)

find_package(catkin REQUIRED roscpp rospy cmake_modules std_msgs tf)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
#add_message_files(FILES  pose_value.msg)
#generate_messages(DEPENDENCIES std_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package( CATKIN_DEPENDS std_msgs)


#############
## includes #
#############
include_directories(SYSTEM)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(/usr/include/)


###########
## Build ##
###########


add_executable(laser src/serial.cpp)
target_link_libraries(laser ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#target_link_libraries(socket_send ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES} /usr/lib/x86_64-linux-gnu/libusb-1.0.so /usr/local/lib/libhidapi-libusb.so)
#############
## Install ##
#############
#catkin_install_python(PROGRAMS scripts/mobot_keyboard.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
