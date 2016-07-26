# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(COUNT_DOWN_LATCH)

set(INCLUDE_TSQUEUE "/home/robotarium/Git/RobotariumRepositories/robotarium/serverBackend/mqttExtravaganza/tracker/mqttclient/countdownlatch/")

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them

# Add OpenCV headers location to your include paths

include_directories(${INCLUDE_TSQUEUE})

add_executable(test ./countdownlatchtest.cpp)

target_link_libraries(test pthread)

set(C++11 "-std=c++11") 

add_definitions(${C++11})
