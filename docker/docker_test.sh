#/bin/bash

# Make sure that 'sudo xhost +' has been run before this script!!!!

docker run --rm -ti \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	-v $2:/tag_tracker/out.mp4 \
	--device=/dev/video0:/dev/video0 \
	$1
