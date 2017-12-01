#/bin/bash

# Make sure that 'sudo xhost +' has been run before this script!!!!

docker run --rm -ti \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	--device=/dev/video0:/dev/video0 \
	tracker_test
