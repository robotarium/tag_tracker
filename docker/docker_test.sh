#!/bin/bash

# Make sure that 'sudo xhost +' has been run before this script!!!!

# Usage: First argument is container name 
# Second argument is absolute path to test video file

docker run --rm -ti \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	-v $2:/tag_tracker/out.mp4 \
	-w /tag_tracker/scripts/ \
	$1 /bin/bash "/tag_tracker/scripts/detect_markers_test.sh" "../out.mp4"
