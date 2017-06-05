# Download and install the Robotarium Tag Tracker, OpenCV and all its dependencies
#This docker setup will allow the tag tracker to run in an isolated enviroment along with all its dependencies
#First we install OpenCV, along with all the required dependencies. This is followed by build the actual tag tracker within the image. 

#First we specify the base image
#FROM ubuntu:latest
FROM ubuntu:16.04

RUN apt-get -y update 
RUN apt-get -y upgrade

RUN apt-get install sudo

#(PAUL) Add a user for use with X11 forwarding
RUN export uid=1000 gid=1000 && \
mkdir -p /home/developer && \
echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
echo "developer:x:${uid}:" >> /etc/group && \
echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
chmod 0440 /etc/sudoers.d/developer && \
chown ${uid}:${gid} -R /home/developer 

# Specify the working directory so that we can navigate back to it. 
#ENV CURRENTDIR /home/TheDocker/
#WORKDIR $CURRENTDIR
ENV PROGRAMDIR /home/developer/programs

# To avoid potential future errors, allow src files to be pulled - add the following lines to sources.list 
#RUN echo 'deb-src http://archive.ubuntu.com/ubuntu trusty main restricted #Added by software-properties' >>/etc/apt/sources.list && \
#    echo 'deb-src http://gb.archive.ubuntu.com/ubuntu/ trusty restricted main universe multiverse #Added by software-properties' >> /etc/apt/sources.list && \
#    echo 'deb-src http://gb.archive.ubuntu.com/ubuntu/ trusty-updates restricted main universe multiverse #Added by software-properties' >> /etc/apt/sources.list && \
#    echo 'deb-src http://gb.archive.ubuntu.com/ubuntu/ trusty-backports main restricted universe multiverse #Added by software-properties' >> /etc/apt/sources.list && \
#    echo 'deb-src http://security.ubuntu.com/ubuntu trusty-security restricted main universe multiverse #Added by software-properties' >> /etc/apt/sources.list && \
#    echo 'deb-src http://gb.archive.ubuntu.com/ubuntu/ trusty-proposed restricted main universe multiverse #Added by software-properties' >> /etc/apt/sources.list && \
 #   apt-get -qq update

#Search for multiverse packages as well
RUN sudo apt-get -y install software-properties-common python-software-properties python3-software-properties
RUN sudo add-apt-repository "multiverse" 

# Install pre-requisites for openCV # (PAUL) removed python-dev, python-numpy
RUN sudo apt-get -qq update && \
    sudo apt-get -y build-dep opencv && \ 
    sudo apt-get -qq -y install libfaac-dev libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config \
                           libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer0.10-dev \
		           libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev \
                           libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip

# Install some required libraries (git has been added to this list)
RUN sudo apt-get -y install build-essential gcc make cmake cmake-gui cmake-curses-gui libssl-dev doxygen graphviz yasm git

# Install wget 
RUN sudo apt-get -y install wget

# Install NASM from source for x264 compatibility 
RUN wget http://www.nasm.us/pub/nasm/releasebuilds/2.13.01/nasm-2.13.01.tar.xz
RUN tar -xvf nasm-2.13.01.tar.xz && \
	cd nasm-2.13.01 && \
	./configure --prefix=/usr && \
	make && \
	make install

# Get out of this directory
RUN cd ../ 

# Install X264 library from source
RUN git clone git://git.videolan.org/x264.git && \
    cd x264 && \
    ./configure --enable-static --enable-shared && \
    make -j $(($(nproc)-1)) && \
    make install && \
    cd ..

# Install liblame. First, download the tar file from the internet.
RUN mkdir /home/developer/programs
RUN cd $PROGRAMDIR && \
	wget https://sourceforge.net/projects/lame/files/lame/3.99/lame-3.99.5.tar.gz && \
    tar -xvzf lame-3.99.5.tar.gz && \
    cd lame-3.99.5 && \
    ./configure && \
    make -j $(($(nproc) - 1)) && \
    make install

# Install FFMpeg library
#RUN git clone https://github.com/FFmpeg/FFmpeg.git && \
#    apt-get -y install libfdk-aac-dev && \
#    cd FFmpeg && \
#    ./configure --enable-nonfree --enable-pic --enable-shared --enable-gpl --enable-pthreads --enable-libx264 --enable-libfdk-aac --enable-libmp3lame && \
#    make -j $(($(nproc) - 1)) && \
#    make install && \
#    cd ..

# (PAUL) Get ffmpeg from standard lib 
RUN sudo apt-get -y install ffmpeg

#(PAUL) We don't use the paho.mqtt.c libraries - just mosquitto
#Install MQTT libraries and client (C)
#RUN git clone https://github.com/eclipse/paho.mqtt.c.git && \
#    cd paho.mqtt.c/build && \
#    cmake .. && \
#    make && \
#    make install && \
#    cd .. && cd ..  #head back to the main directory

#(PAUL) I don't think we use the cpp libraries...
#RUN git clone https://github.com/eclipse/paho.mqtt.cpp.git && \
#    cd paho.mqtt.cpp && \
#    make && \
#    cp lib/libmqttpp.so.0.1 /usr/local/lib/ && \
#    cd /usr/local/lib/ && \
#    ln -s libmqttpp.so.0.1 libmqttpp.so.0 && \
#    ln -s libmqttpp.so.0 libmqttpp.so && \
#    cd $CURRENTDIR

#Install Boost Libraries
RUN sudo apt-get -y install libboost1.58-dev libboost-all-dev

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
    #cd $CURRENTDIR #Navigate back to main directory

# Install Image Libraries
RUN sudo apt-get -y install libpng12-0 libpng12-dev libavcodec-dev libavcodec-extra libavformat-dev libavresample-dev libavutil-dev libswscale-dev && \
    sudo apt-get -y install libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev

#Install video driver libraries
RUN sudo apt-get -y install libv4l-0 libv4l-dev mencoder libv4l-0 libv4l-dev libv4lconvert0 v4l-conf v4l-utils v4l2ucp libogg0 libogg-dev libtbb2 \
               libtbb-dev libx264-148 libx264-dev libdc1394-22 libfdk-aac0 libfdk-aac-dev

#Install the required audio codecs 
RUN sudo apt-get install libmp3lame0

#Install OpenGL libraries
RUN sudo apt-get install libgl1-mesa-dev mesa-common-dev

# Install numeric libraries
RUN sudo apt-get -y install liblapack-dev libblas3 libblas-dev liblapack3 liblapacke libatlas3-base libeigen3-dev

#Download the OpenCV contrib libraries
RUN cd $PROGRAMDIR && \
	git clone https://github.com/opencv/opencv_contrib.git

# Compile OpenCV
RUN cd $PROGRAMDIR && \
	git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_EXTRA_MODULES_PATH=/home/developer/programs/opencv_contrib/modules -D BUILD_opencv_python3=OFF -D \ 
    BUILD_PYTHON_SUPPORT=OFF -D WITH_TBB=ON -D WITH_V4L=ON -D BUILD_EXAMPLES=OFF -D BUILD_NEW_PYTHON_SUPPORT=OFF -D WITH_GSTREAMER=OFF -D \ 
    WITH_OPENGL=ON -D WITH_QT=OFF .. && \
    make -j $(($(nproc) - 1)) && \
    make install 

# Install Mosquitto 
RUN cd $PROGRAMDIR && \
	sudo apt-get -y install libc-ares-dev uuid-dev xsltproc && \
    wget http://mosquitto.org/files/source/mosquitto-1.4.10.tar.gz && \
    tar -xvzf mosquitto-1.4.10.tar.gz && \
    cd mosquitto-1.4.10 && \
    make -j $(($(nproc) - 1)) && \
    make install
    #cd $CURRENTDIR   

#RUN apt-get -y install libc-ares-dev uuid-dev xsltproc && \ 
#    git clone https://github.com/eclipse/mosquitto.git && \
#    cd mosquitto && \
#    make && \
#    make install && \
#    cd $CURRENTDIR

# Now that openCV is installed, download the tag tracker and compile it!
RUN cd $PROGRAMDIR && \
	git clone https://github.com/robotarium/tag_tracker.git && \
    cd tag_tracker && \
	git checkout Dockerization && \
    mkdir build && cd build && \
    cmake .. && \
    make -j $(($(nproc) - 2)) 

RUN sudo apt-get install -y libcanberra-gtk-module libcanberra-gtk3-module

USER developer
ENV HOME /home/developer

CMD cd $PROGRAMDIR/tag_tracker/scripts && \
	sudo ./detectMarkers.sh


