cmake_minimum_required(VERSION 2.8.3)
project(reactive_move_controller)

find_package(catkin REQUIRED COMPONENTS
	roscpp 
	nav_core
	tf
	angles
	sensor_msgs
	geometry_msgs
	pluginlib
	visualization_msgs
)

find_package(PkgConfig REQUIRED)

#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS 
	roscpp 
	nav_core
	tf
	angles
	sensor_msgs
	geometry_msgs
	pluginlib
	visualization_msgs
    LIBRARIES 
    INCLUDE_DIRS 
)

include_directories(${catkin_INCLUDE_DIRS} )

#add_library()

#add_dependencies(sick_rfu_620 ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})


add_executable(reactive_move_controller
    src/main.cpp)

target_link_libraries(reactive_move_controller
    ${catkin_LIBRARIES}
)
#target_link_libraries()
