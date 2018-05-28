cmake_minimum_required(VERSION 3.8)
project(sppech_based_teleop_commander)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        ros_verbal_interaction_node/src/speech_cagg_manager.cpp)

add_executable(sppech_based_teleop_commander ${SOURCE_FILES})