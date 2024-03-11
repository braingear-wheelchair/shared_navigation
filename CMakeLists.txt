set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(shared_navigation)
find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs 
			 message_generation 
			 tf 
			 dynamic_reconfigure
			 proximity_grid)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
	cfg/SharedNavigation.cfg
)


catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
)

add_library(${PROJECT_NAME} 
			src/SharedNavigation.cpp
			)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(navigation src/navigation.cpp)
target_link_libraries(navigation ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(navigation ${PROJECT_NAME}_gencfg)

