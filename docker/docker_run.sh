#!/bin/bash

# Make sure that 'sudo xhost +' has been run before this script!!!!

# Usage: First argument is container name 
# Second argument is absolute path to test video file

docker run --rm -ti \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	--device /dev/video0:/dev/video0 \
	-v /home/robotarium/git/tag_tracker:/tag_tracker \
	robotarium:tag_tracker
