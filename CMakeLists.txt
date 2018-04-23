cmake_minimum_required(VERSION 2.8.3)
project(sas_space_explore)

add_definitions(-DBOOST_ALL_DYN_LINK)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 ${CMAKE_CXX_FLAGS}")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    grid_map_ros
    internal_grid_map
    nav_msgs
    opt_utils
    car_model
    roscpp
    roslib
    rospy
    rosparam_handler
    dynamic_reconfigure
)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED yaml-cpp > 0.5)


# Generate parameter files
generate_ros_parameter_files(cfg/explore.params)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS grid_map_ros internal_grid_map nav_msgs opt_utils car_model roscpp
                   roslib rospy rosparam_handler dynamic_reconfigure
    DEPENDS OpenCV
)


###########
## Build ##
###########

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/sas_explore/sas_explore.cpp
        src/sas_explore/motion_primitive.cpp
)
  
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)
  
set(linked_libs
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/sas_explore/sas_explore_node.cpp src/parameters_configure/parameters.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_genparam)
target_link_libraries(${PROJECT_NAME}_node
    ${linked_libs}
)

add_executable(get_plan_server_node
        src/sas_explore/get_plan_server_node.cpp)
target_link_libraries(get_plan_server_node
        ${linked_libs})

