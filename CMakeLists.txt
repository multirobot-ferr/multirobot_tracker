cmake_minimum_required(VERSION 2.8.3)
project(multirobot_tracker)

find_package(catkin REQUIRED
            COMPONENTS
                roscpp
                message_generation
                std_msgs
            )


generate_messages(
    DEPENDENCIES
    std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(multirobot_tracker src/multirobot_tracker_node.cpp)
target_link_libraries(multirobot_tracker ${catkin_LIBRARIES})
add_dependencies(multirobot_tracker ${PROJECT_NAME}_generate_messages_cpp)
