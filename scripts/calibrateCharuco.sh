# ===================
# 	USAGE:
# ===================
# These flags are required to run in a basic configuration
# -w=5		number of charuco markers in x-direction of board
# -h=4		number of charuco markers in y-direction of board
# -d=0		dictionary ID for generated marker
# --ml=100	marker side length in pixels
# --sl=40 	separation between two markers in pixels
# --dp=file	file containing detector parameters
#
# The following optional flags are available
# --ci=0	camera index (e.g. /dev/video0, by default 0)
# --sc 		show detected chess board corners after detection
# --rs 		apply refined strategy
# --pc 		fix the principal point at the center

# ======================
# OpenCV 2.4.0 and lower
# ======================
# Newest script for 5 x 4 charuco board
#../build/calibrateCameraCharuco -w 5 -h 4 -sl 0.05 -ml 0.04 -d 0 -ci 0 -sc -rs -dp ../detector_params.yml -o cameraCalibration.yml

# =======================
# OpenCV 3.0.0 and higher
# =======================
# Newest script for 5 x 4 charuco board
../build/calibrateCameraCharuco -w=5 -h=4 -sl=0.05 -ml=0.04 -d=0 -ci=0 -sc -rs -dp=../detector_params.yml cameraCalibration1.yml

# ================================
# Old commands for OpenCV <= 2.4.0
# ================================
#./calibrateCameraCharuco -w 5 -h 3 -sl 0.036 -ml 0.025 -d 0 -ci 1 -dp ../detector_params.yml -sc -o cameraCalibration.yml
#../build/calibrateCameraCharuco -w 5 -h 3 -sl 0.036 -ml 0.025 -d 0 -ci 1 -sc -o cameraCalibration.yml
#../build/calibrateCameraCharuco -w 5 -h 4 -sl 0.04 -ml 0.026 -d 0 -ci 1 -sc -o cameraCalibration.yml
