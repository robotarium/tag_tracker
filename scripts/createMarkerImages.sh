# ===================
# 	USAGE:
# ===================
# These flags are required to run in a basic configuration
# -d=0		dictionary ID for generated marker
# --id=1	id encoded into marker
# 
# The following optional flags are available
# --bb 		number of pixels in marker border
# --si		show generated marker image

# ======================
# OpenCV 2.4.0 and lower
# ======================
#./createMarker -d 0 -o marker.jpg -id 2 -bb 1 -si

# =======================
# OpenCV 3.0.0 and higher
# =======================
./createMarker -d=0 --id=2 --bb=1 --si marker.jpg 
