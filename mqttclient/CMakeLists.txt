# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(MQTT_CLIENT)

set(INCLUDE_TSQUEUE "/home/robotarium/Git/RobotariumRepositories/robotarium/serverBackend/mqttExtravaganza/tracker/mqttclient/tsqueue/")
set(INCLUDE_PROMISE "./promise")
set(INCLUDE_CLIENT "./src/")

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them

# Add OpenCV headers location to your include paths
                  
include_directories(${INCLUDE_TSQUEUE} ${INCLUDE_PROMISE} ${INCLUDE_CLIENT})

add_executable(mqttclienttest ./test/mqttclienttest.cpp)

#add_library(mqttclient /home/robotarium/Git/RobotariumRepositories/robotarium/serverBackend/mqttExtravaganza/tracker/mqttclient/src/mqttclient.cpp)

#add_library(tsqueue /home/robotarium/Git/RobotariumRepositories/robotarium/serverBackend/mqttExtravaganza/tracker/mqttclient/tsqueue/tsqueue.cpp)

#target_link_libraries(mqttclient tsqueue)

target_link_libraries(mqttclienttest pthread)
#target_link_libraries(mqttclienttest mosquitto)
#target_link_libraries(mqttclienttest paho-mqtt3c)
#target_link_libraries(mqttclienttest paho-mqtt3cs)
target_link_libraries(mqttclienttest mqttpp)
target_link_libraries(mqttclienttest paho-mqtt3a)
set(C++11 "-std=c++11") 

add_definitions(${C++11})
