# ===================
# 	USAGE:
# ===================
# These flags are required to run in a basic configuration
# -w=5		number of charuco markers in x-direction of board
# -h=4		number of charuco markers in y-direction of board
# -d=0		dictionary ID for generated marker
# -l=100	marker side length in pixels
# -s=40 	separation between two markers in pixels
# 
# The following optional flags are available
# --si 		show generated output image
#
# Note that dimensions of square and marker are in pixels (options -sl and -ml)

# ======================
# OpenCV 2.4.0 and lower
# ======================
#../build/createBoard -w 5 -h 5 -o board.jpg -l 100 -s 40 -d 0

# =======================
# OpenCV 3.0.0 and higher
# =======================
../build/createBoard -w=5 -h=4 -l=100 -s=40 -d=0 board.jpg 

