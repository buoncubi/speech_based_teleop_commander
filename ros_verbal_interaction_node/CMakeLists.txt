cmake_minimum_required(VERSION 3.8)
project(ros_verbal_interaction_node)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/speech_cagg_manager.cpp)

add_executable(ros_verbal_interaction_node ${SOURCE_FILES})