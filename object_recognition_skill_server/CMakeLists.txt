cmake_minimum_required(VERSION 2.8.3)
project(object_recognition_skill_server)

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    actionlib
    dynamic_robot_localization
    object_recognition_skill_msgs
    rosconsole
    roscpp
    tf2
    tf2_geometry_msgs
)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/object_recognition_skill_server.cpp
)

add_executable(${PROJECT_NAME}_node
    src/object_recognition_skill_server_node.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
