# ===================
# 	USAGE:
# ===================
# First argument: path to video file for testing 

../build/detectMarkers -m -sd=true -ad=true -bb=true -s=1.0 -p=1884 -d=0 -ci=0 -v=$1 -c=../data/camera.yml -l=0.026 -dp=../data/detector_params.yml -rm=../data/reference_markers_setup.yml
