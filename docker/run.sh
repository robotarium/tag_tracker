#/bin/bash 

docker run -ti --rm \
	-e DISPLAY=$DISPLAY \
	-u developer \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	--privileged \
	tracker

