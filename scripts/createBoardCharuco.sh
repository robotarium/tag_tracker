# ===================
# 	USAGE:
# ===================
# These flags are required to run in a basic configuration
# -w=5		number of charuco markers in x-direction of board
# -h=4		number of charuco markers in y-direction of board
# -d=0		dictionary ID for generated marker
# --id=1	id encoded into marker
# --ml=800	marker side length in pixels
# --sl=1000 	square side length in pixels
# 
# The following optional flags are available
# --si 		show generated output image
#
# Note that dimensions of square and marker are in pixels (options -sl and -ml)

# ======================
# OpenCV 2.4.0 and lower
# ======================
#../build/createBoardCharuco -w 5 -h 4 -o charucoBoard.jpg -d 0 -sl 1000 -ml 800

# =======================
# OpenCV 3.0.0 and higher
# =======================
../build/createBoardCharuco -w=5 -h=4 -d=0 -sl=1000 -ml=800 charucoBoard.jpg
