# Download and install the Robotarium Tag Tracker, OpenCV and all its dependencies
#This docker setup will allow the tag tracker to run in an isolated enviroment along with all its dependencies
#First we install OpenCV, along with all the required dependencies. This is followed by build the actual tag tracker within the image.

#First we specify the base image
FROM ubuntu:16.04

# Requisite upgrades
RUN apt-get -y update
RUN apt-get -y upgrade

# (PAUL) Get ffmpeg from standard lib
#RUN sudo apt-get -y install ffmpeg

# Install OpenCV

# Install all dependencies for OpenCV 3.2
RUN apt-get -y update && apt-get -y install python3.5-dev python3-pip wget unzip \
    build-essential cmake git pkg-config libatlas-base-dev gfortran \
    libjasper-dev libgtk2.0-dev libavcodec-dev libavformat-dev \
    libswscale-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libv4l-dev \
    && pip3 install numpy

# Get OpenCV contrib for ArUco
RUN wget https://github.com/Itseez/opencv/archive/3.2.0.zip -O opencv3.zip \
    && unzip -q opencv3.zip && mv /opencv-3.2.0 /opencv && rm opencv3.zip \
    && wget https://github.com/Itseez/opencv_contrib/archive/3.2.0.zip -O opencv_contrib3.zip \
    && unzip -q opencv_contrib3.zip && mv /opencv_contrib-3.2.0 /opencv_contrib && rm opencv_contrib3.zip

# prepare build
RUN mkdir /opencv/build && cd /opencv/build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D BUILD_PYTHON_SUPPORT=OFF \
      -D BUILD_JAVA_SUPPORT=OFF \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_opencv_apps=OFF \
      -D BUILD_DOCS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_TESTS=OFF \
      -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3 \
      -D BUILD_opencv_python3=OFF \
      -D BUILD_opencv_python2=OFF \
      -D WITH_IPP=OFF \
      -D WITH_FFMPEG=ON \
      -D WITH_V4L=ON ..

# Compile and install OpenCV
RUN cd /opencv/build && make -j $(($(nproc) - 1)) && make install && ldconfig

# We need this for visualization
RUN apt-get install -y libcanberra-gtk-module libcanberra-gtk3-module

#Install JSON Libraries

RUN cd $PROGRAMDIR && \
  	git clone https://github.com/nlohmann/json.git && \
    cd json && \
    mkdir build && \
    cd build/ && \
    cmake .. && \
    make -j $(($(nproc) - 1)) && \
    make install && \
    ln -s /usr/local/include/nlohmann/json.hpp /usr/include/json.hpp

# Install Mosquitto

RUN cd $PROGRAMDIR && \
    apt-get -y install libssl-dev libc-ares-dev uuid-dev xsltproc && \
    wget http://mosquitto.org/files/source/mosquitto-1.4.10.tar.gz && \
    tar -xvzf mosquitto-1.4.10.tar.gz && \
    cd mosquitto-1.4.10 && \
    make -j $(($(nproc) - 1)) && \
    make install


# general purpose stuffs
RUN apt-get -y install vim

# Add this line to prevent caching
ENV updated-adds-on 06-21-2017-4

# Now that openCV is installed, download the tag tracker and compile it!
RUN git clone https://github.com/robotarium/tag_tracker.git && \
    cd tag_tracker && \
    git checkout Dockerization && \
    mkdir build && cd build && \
    cmake .. && \
    make -j $(($(nproc) - 2))
