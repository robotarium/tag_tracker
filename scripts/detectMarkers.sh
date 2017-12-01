# ===================
# 	USAGE:
# ===================
# These flags are required to run in a basic configuration
# - d 0 	the dictionary for the robot tags (id 0)
# - ci 0 	the ID of the camera (e.g. /dev/video0 in this case)
# - c 		the camera calibration file
# - l 0.036 	the size of the aruco marker in meters
#
# The following optional flags are available
# - vo 		flag that determines whether output video is recorded or not
# - rM 		flag that determines whether markers are rendered into output video
# - m		use metric coordinates in output messages (versus pixel coordinates)

../build/detectMarkers -m -sd=true -ad=true -bb=true -s=1.0 -p=1884 -d=0 -ci=0 -v=/home/robotarium/Desktop/out.mp4 -c=../data/camera.yml -l=0.026 -dp=../data/detector_params.yml -rm=../data/reference_markers_setup.yml
