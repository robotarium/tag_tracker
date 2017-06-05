#/bin/bash 

# Make sure that 'xhost +' has been run before this script!!!!

docker run -ti --rm \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	-v /home/robotarium/Git/RobotariumRepositories/robotarium/serverBackend/mqttExtravaganza/tracker_alias.json:/home/robotarium/Git/RobotariumRepositories/robotarium/serverBackend/mqttExtravaganza/tracker_alias.json \
	--device=/dev/video0:/dev/video0 \
	tracker

