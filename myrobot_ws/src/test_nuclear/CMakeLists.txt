cmake_minimum_required(VERSION 2.8.3)
project(test_nuclear)
add_compile_options(-std=c++11)
 
find_package(catkin REQUIRED COMPONENTS roscpp rospy serial std_msgs sensor_msgs)
find_package(Boost REQUIRED system filesystem date_time thread)


###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS roscpp rospy serial std_msgs sensor_msgs nav_msgs geometry_msgs)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})

#add_executable(serial_node src/serial_node.cpp)
#add_dependencies(serial_node test_nuclear_messages_cpp)
#target_link_libraries(serial_node ${catkin_LIBRARIES})

#add_executable(save_file src/save_file.cpp)
#add_dependencies(save_file test_nuclear_messages_cpp)
#target_link_libraries(save_file ${catkin_LIBRARIES})

add_executable(nuclear_interface src/nuclear_interface.cpp)
add_dependencies(nuclear_interface test_nuclear_messages_cpp)
target_link_libraries(nuclear_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(nuclear_coordinator src/nuclear_coordinator.cpp)
add_dependencies(nuclear_coordinator test_nuclear_messages_cpp)
target_link_libraries(nuclear_coordinator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(navigation_generator src/navigation_generator.cpp)
add_dependencies(navigation_generator test_nuclear_messages_cpp)
target_link_libraries(navigation_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})





