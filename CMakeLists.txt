#cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(cameraCalibration)

set (INCLUDE_ROBOTARIUM "../messageTypes/" )
set (INCLUDE_LOCAL_LIBS "libpaho-mqtt3a;libpaho-mqtt3as")
set(INCLUDE_TSQUEUE "mqttclient/tsqueue/")
set(INCLUDE_PROMISE "mqttclient/promise/")
set(INCLUDE_CLIENT "mqttclient/src/")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS}
  ${INCLUDE_ROBOTARIUM} ${INCLUDE_CLIENT} ${INCLUDE_PROMISE} ${INCLUDE_TSQUEUE})

# Declare the executable target built from your sources
add_executable(detectMarkers ./src/detect_markers_opencv_3.0.0.cpp)

# Link your application with OpenCV libraries

target_link_libraries(detectMarkers ${OpenCV_LIBS})
target_link_libraries(detectMarkers mosquitto)
target_link_libraries(detectMarkers pthread)
target_link_libraries(detectMarkers paho-mqtt3c)
target_link_libraries(detectMarkers mqttpp)
target_link_libraries(detectMarkers paho-mqtt3a)

set(C++11 "-std=c++11")

add_definitions(${C++11})
