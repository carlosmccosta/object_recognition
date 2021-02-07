cmake_minimum_required(VERSION 3.0.2)
project(object_recognition_skill_msgs)


set(${PROJECT_NAME}_CATKIN_COMPONENTS
    message_generation
)

set(${PROJECT_NAME}_CATKIN_COMPONENTS_MSGS
    actionlib_msgs
    geometry_msgs
    std_msgs
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_COMPONENTS}
             ${${PROJECT_NAME}_CATKIN_COMPONENTS_MSGS})

add_action_files(
    FILES
        ObjectRecognitionSkill.action
)

generate_messages(
    DEPENDENCIES
        ${${PROJECT_NAME}_CATKIN_COMPONENTS_MSGS}
)

catkin_package(
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        ${${PROJECT_NAME}_CATKIN_COMPONENTS_MSGS}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
