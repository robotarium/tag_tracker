/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

/* Include opencv libraries */
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <algorithm>
#include <chrono>

/* For Rodrigues transformation */
#include <opencv2/calib3d/calib3d.hpp>

/* Include IO functionalities */
#include <json.hpp>
#include <iostream>
#include <fstream>
#include <mqttclient.hpp>

/* Include threading for use in threaded detection */
#include <future>

/* Chrono typedefs */
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

/* Convenience declaration */
using json = nlohmann::json;

/* Name spaces */
using namespace std;
using namespace cv;

/* Forward declarations */

static void onMouse(int event, int x, int y, int, void*);

bool find_homography_to_reference_markers_image_plane(
  VideoCapture& cam,
  Ptr<aruco::Dictionary> dictionary,
  Ptr<aruco::DetectorParameters> detectorParams,
  const vector< int > reference_marker_ids,
  const vector< Point2f > reference_markers_image_plane_WORLD_PLANE,
  Mat& H);

bool readReferenceMarkersSpecs(string reference_marker_setup_file, vector<int>& referenceMarkersIDs, vector<Point2f>& referenceMarkersPositions);

/* End forward declarations */

/* Global variables */

json powerData;
json status_data;
std::map<std::string, vector<double>> robotImagePosition;
std::map<std::string, bool> closeToGritsbot, clickedOnGritsbot;
std::ostringstream ss;

/* Tracker parameters */
int dictionaryId;
bool showRejected;
bool estimatePose;
float markerLength;
int camId;
std::string video_file;
Mat camMatrix, distCoeffs, projMatrix;

/* OpenCV video input and output devices */
VideoCapture inputVideo;

/* Framesize */
int frameWidth  = 1280;
int frameHeight = 720;

/* End global variables */

cv::Size frameSize = cv::Size(frameWidth, frameHeight);

/* Function that generates a dictionary with a single marker from a base dictionary */
Ptr<aruco::Dictionary> generate_single_dictionary(int marker, int marker_size,
                                           const Ptr<aruco::Dictionary> &base_dictionary) {

    Ptr<aruco::Dictionary> out = makePtr<aruco::Dictionary>();
    out->markerSize = marker_size;

    // theoretical maximum intermarker distance
    // See S. Garrido-Jurado, R. Muñoz-Salinas, F. J. Madrid-Cuevas, and M. J. Marín-Jiménez. 2014.
    // "Automatic generation and detection of highly reliable fiducial markers under occlusion".
    // Pattern Recogn. 47, 6 (June 2014), 2280-2292. DOI=10.1016/j.patcog.2014.01.005
    int C = (int)std::floor(float(marker_size * marker_size) / 4.f);
    int tau = 2 * (int)std::floor(float(C) * 4.f / 3.f);

    // if base_dictionary is provided, calculate its intermarker distance
    if(base_dictionary->bytesList.rows > 0) {
        CV_Assert(base_dictionary->markerSize == marker_size);
        out->bytesList = base_dictionary->bytesList.rowRange(marker, marker+1).clone();
    }

    // update the maximum number of correction bits for the generated dictionary
    out->maxCorrectionBits = (tau - 1) / 2;

    return out;
}

/* -------------------------------------
 *		MQTT Callback functions
 * ------------------------------------- */
void powerDataCallback(std::string topic, std::string message) {
  bool debug = false;

  /* Parse ID from topic
   * topic names are of the form ID/power_data
   */
  int id = -1;
  std::string tmp;
  std::stringstream ss(topic);
  ss >> id >> tmp;

	if(debug) {
		std::cout << "Topic				: " << topic << std::endl;
		std::cout << "Message			: " << message << std::endl;
		std::cout << "Extracted ID: " << id << std::endl;
	}

  /* Parse message string into JSON dictionary */
  try {
	auto data = json::parse(message);

	/* Write battery voltage to global powerData JSON dictionary */
	/* Check if an entry vBat exists in the message */
	if (data.find("vBat") != data.end()) {
			if(debug) {
			     std::cout << "Power data in callback: " << data["vBat"] << std::endl;
			}

			/* Check if robot id has been extracted from topic */
			if(id >= 0) {
			     powerData[std::to_string(id)] = data["vBat"];
			}
	}

  /* Write charging status to global powerData JSON dictionary */
  /* Check if an entry vBat exists in the message */
  if (data.find("charging") != data.end()) {
      if(debug) {
           std::cout << "Power data in callback: " << data["charging"] << std::endl;
      }

      /* Check if robot id has been extracted from topic */
      if(id >= 0) {
        status_data[std::to_string(id)] = ((float) data["charging"] > 0.0);
      }
  }
  } catch (const std::exception& e) {

  }
}

/* Cast function to standard function pointer */
std::function<void(std::string, std::string)> stdf_powerDataCallback = &powerDataCallback;

/* --------------------------------------------------
 *		Camera/tracker configuration functions
 * -------------------------------------------------- */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs, Mat &projMatrix) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs["projection_matrix"] >> projMatrix;
    return true;
}

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    //fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

/* --------------------------------------
 *		Command line help text
 * -------------------------------------- */
namespace {
	const char* about = "Basic marker detection";
	const char* keys  =
	        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
	        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
	        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
	        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
	        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
	        "{v         |      | Video file (takes precedence over ci) }"
	        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
	        "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
	        "{dp       |       | File of marker detector parameters }"
          "{bb       | false | Whether to use bounding boxes }"
          "{ad       | false | Whether to use threading in detection }"
          "{sd       | false | Whether to use single dictionaries in detection}"
	        "{r        |       | show rejected candidates too }"
			    "{h        | | MQTT broker }"
			    "{p        | 1883      | MQTT port }"
			    "{s        | 1.0  | frame scale: if specified, frame sizes are multiplied by s }"
			    "{rm        |  | reference markers: reference markers yml specification file }";
}

static void onMouse(int event, int x, int y, int, void*) {
    switch (event) {
    case EVENT_MOUSEMOVE:
        for (map<string, vector<double> >::iterator id = robotImagePosition.begin(); id != robotImagePosition.end(); id++) {
            int u = robotImagePosition[id->first][0];
            int v = robotImagePosition[id->first][1];
            if (abs(x - u) < 30 && abs(y - v) < 30)
                closeToGritsbot[id->first] = true;
            else
                closeToGritsbot[id->first] = false;
        }
        break;
    case EVENT_LBUTTONDOWN:
        for (map<string, vector<double> >::iterator id = robotImagePosition.begin(); id != robotImagePosition.end(); id++) {
            if (closeToGritsbot[id->first]) {
                if (!clickedOnGritsbot[id->first])
                    clickedOnGritsbot[id->first] = true;
                else
                    clickedOnGritsbot[id->first] = false;
            }
        }
        break;
    default:
        break;
    }
}

/* Finds a homography to a reference plane by looking for reference markers */
bool find_homography_to_reference_markers_image_plane(
  VideoCapture& cam,
  Ptr<aruco::Dictionary> dictionary,
  Ptr<aruco::DetectorParameters> detectorParams,
  const vector<int> reference_marker_ids,
  const vector<Point2f> reference_markers_world_plane,
  Mat& H) {

  Mat img;
  vector<vector<Point2f>> corners, rejected, reference_markers_image_plane;
  Mat2f reference_markers_image_plane_toundistort = Mat2f::zeros(reference_markers_world_plane.size(), 1),
        reference_markers_image_plane_undistorted = Mat2f::zeros(reference_markers_world_plane.size(), 1);
  std::vector< int > ids;

  while(true) {

    try {

      cam >> img;

      /* Reference markers could be anywhere in the image, so look at the whole thing */
      aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);

      if (ids.size() > 0) {
        /* Clear the reference markeers in the image plane */
        reference_markers_image_plane.clear();

        for (int i = 0; i < reference_marker_ids.size(); i++) {
          vector<int>::iterator iter = find(ids.begin(), ids.end(), reference_marker_ids[i]);
          if (iter != ids.end()) {
            int idx = distance(ids.begin(), iter);
            reference_markers_image_plane.push_back(corners[idx]);
          }
        }

        /* Get the center of the reference markers */
        if (reference_markers_image_plane.size() == reference_marker_ids.size()){
          vector< Point2f > image_points, world_points;
          for (int i = 0; i < reference_marker_ids.size(); i++) {
            reference_markers_image_plane_toundistort[i][0][0] = 0.25*(reference_markers_image_plane[i][0].x+reference_markers_image_plane[i][1].x+reference_markers_image_plane[i][2].x+reference_markers_image_plane[i][3].x);
            reference_markers_image_plane_toundistort[i][0][1] = 0.25*(reference_markers_image_plane[i][0].y+reference_markers_image_plane[i][1].y+reference_markers_image_plane[i][2].y+reference_markers_image_plane[i][3].y);
          }

          undistortPoints(
            reference_markers_image_plane_toundistort,
            reference_markers_image_plane_undistorted,
            camMatrix,
            distCoeffs,
            Mat::eye(Size(3,3), CV_32F),
            projMatrix
          );

          /* Store reference markers and world points for homography */
          for (int i = 0; i < reference_marker_ids.size(); i++){
            image_points.push_back(
              Point2f(
                reference_markers_image_plane_undistorted[i][0][0],
                reference_markers_image_plane_undistorted[i][0][1]
              )
            );
            world_points.push_back(reference_markers_world_plane[i]);
          }

          /* Find homography using the pre-specified points */
          H = findHomography(image_points, world_points);

     	    putText(img, "Reference markers", Point2f(img.cols*0.1, img.rows*0.4), FONT_HERSHEY_COMPLEX, int(3*float(frameWidth)/1280), Scalar(0, 127, 255), 2);
          putText(img, "found", Point2f(img.cols*0.35, img.rows*0.6), FONT_HERSHEY_COMPLEX, int(3*float(frameWidth)/1280), Scalar(0, 127, 255), 2);
          imshow("out", img);
	        waitKey(333);

          return true;
        }

        cv::aruco::drawDetectedMarkers(img, corners, ids);
      }

      /* Display something on the image to indicate that we're searching for reference markers */
      putText(img, "Searching for", Point2f(img.cols*0.2, img.rows*0.4), FONT_HERSHEY_COMPLEX, int(3*float(frameWidth)/1280), Scalar(0, 127, 255), 2);
      putText(img, "reference markers", Point2f(img.cols*0.1, img.rows*0.6), FONT_HERSHEY_COMPLEX, int(3*float(frameWidth)/1280), Scalar(0, 127, 255), 2);

      for (int i = 0; i < reference_marker_ids.size(); i++) {
        ss.str("");
	      ss.clear();
	      ss << reference_marker_ids[i];
	      const String idStr = ss.str();
        putText(img, idStr, Point2f(img.cols*(0.05+0.8*int(i<=1)), img.rows*(0.15+0.8*int(i<1||i>2))), FONT_HERSHEY_COMPLEX, int(3*float(frameWidth)/1280), Scalar(0, 127, 255), 2);
      }

      imshow("out", img);

      char key = (char)waitKey(10);
      if(key == 27) break;

    } catch (int e) {
      // TODO: Exception handling
    }
  }
  return false;
}

/* Reads in reference marker IDs and locations from a .yml file */
bool readReferenceMarkersSpecs(string reference_marker_setup_file, vector<int>& referenceMarkersIDs, vector<Point2f>& referenceMarkersPositions) {
    FileStorage fs(reference_marker_setup_file, FileStorage::READ);
    if(!fs.isOpened())
        return false;

	FileNode markers = fs["markers"];
	for (FileNodeIterator it = markers.begin(); it != markers.end(); it++ ) {
		referenceMarkersIDs.push_back((int)(*it)["id"]);
		referenceMarkersPositions.push_back(Point2f((float)(*it)["x"],(float)(*it)["y"]));
	}

    return true;
}

/* --------------------------------------
 *		        MAIN FUNCTION
 * -------------------------------------- */
int main(int argc, char *argv[]) {

  /* The overall detection algorithm switches between two states.

    0: The algorithm scans the whole image for markers, storing ones that it finds.

    1: The algorithm looks in a bounding box only for markers that it detected
    in state 0.  If it doesn't see one of these markers, it goes back to state
    0 to look for it.

	/* Initialize command line parsing */
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);

  /* Set up MQTT client.  Make this a shared pointer so that we don't HAVE to use MQTT */
  std::shared_ptr<MQTTClient> m;
  if(parser.has("h")) {
    m = std::make_shared<MQTTClient>(parser.get<string>("h"), parser.get<int>("p"));
    // Start the MQTT client's internal threading
    m->start();
  }

	/* Error handling for missing command line parameters */
	if(argc < 2) {
		parser.printMessage();
		return 0;
	}

	if(!parser.check()) {
		parser.printErrors();
		return 0;
	}

	/* Read in basic detector parameters */
	dictionaryId = parser.get<int>("d");
	showRejected = parser.has("r");

  if(!parser.has("c")) {
    std::cout << "Need to provide camera intrinsic parameters!" << std::endl;
    return 0;
  }

  /* Get marker length from arguments */
	markerLength = parser.get<float>("l");

  /* Check for input from video file, rather than camera */
  if(parser.has("v")) {
    video_file = parser.get<std::string>("v");
    /* Open camera input stream */
    inputVideo.open(video_file);
  } else {
    camId = parser.get<int>("ci");
    inputVideo.open(camId);
  }

	/* Initialize detector parameters from file */
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	if(parser.has("dp")) {
		if(!readDetectorParameters(parser.get<string>("dp"), detectorParams)) {
			cerr << "Invalid detector parameters file" << endl;
			return 0;
		}
	}

  /* Get reference marker ids from given reference marker files */
  vector<int> reference_marker_ids;
  vector<Point2f> reference_markers_world_plane;
  if (!readReferenceMarkersSpecs(parser.get<string>("rm"), reference_marker_ids, reference_markers_world_plane)) {
      cerr << "Invalid reference markers file" << endl;
      return 0;
  }

	/* Instantiate dictionary containing tags */
	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	if(!readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs, projMatrix)) {
		cerr << "Invalid camera file" << endl;
		return 0;
	}

  /* Set resolution */
	if (parser.has("s")) {
		frameWidth = int( float(frameWidth) * parser.get<float>("s") );
		frameHeight = int( float(frameHeight) * parser.get<float>("s") );
	}
	frameSize = cv::Size(frameWidth, frameHeight);
	inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, frameWidth);
	inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight);

	std::cout << "Image size: " << frameSize.width
						<< " / " << frameSize.height << std::endl;

  /* Whether to use bounding boxes */
  bool use_boxes = parser.get<bool>("bb");

  /* Whether to use threading */
  bool async_detection = parser.get<bool>("ad");

  /* Use singleton dictionaries */
  bool use_single_dictionaries = parser.get<bool>("sd");

	/* Set MQTT publishing topic */
	std::string main_publish_channel = "overhead_tracker/all_robot_pose_data";

	/* Subscribe to power data channels */
  if(parser.has("h")) {
  	for (int index = 0; index < 50; index++) {
  		/* Create topic name string */
  		std::string topicName = std::to_string(index) + "/power_data";

  		/* Subscribe to MQTT channel */
  		m->subscribe(topicName, stdf_powerDataCallback);
  	}
  }

  /* Create a named output window for display purposes */
  namedWindow("out", 1);
  setMouseCallback("out", onMouse, 0);

  /* Homography for transferring image coordinates to world coordinates.  We also
  Calculate the inverse of this for mapping in the other direction
  */
  Mat H;
  if (!find_homography_to_reference_markers_image_plane(
    inputVideo,
    dictionary,
    detectorParams,
    reference_marker_ids,
    reference_markers_world_plane,
    H
  ))
    return 0;

  /* Pre-calculate the inverse of the homography matrix */
  Mat H_inv = H.inv();

  /* Maps for robot poses and dictionaries for fast lookup */
  std::map<int, cv::Point2f> robot_poses;
  std::map<int, Ptr<aruco::Dictionary>> single_dictionaries;

  /* State machine states.  There are just two.
  0: Look over the whole image for markers
  1: Only look in bounding boxes for images detected in state 0
  */
  int state = 0;
  int next_state = 0;

  auto current_time = Time::now();

  /* Main event loop */
  while(inputVideo.grab()) {
    /* Retrieve new frame from camera */
    Mat image, imageCopy;
    inputVideo.retrieve(image);
    /* Render results */
    // resize(image, imageCopy, Size(), resize_factor, resize_factor, CV_INTER_LINEAR);
    image.copyTo(imageCopy);

    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;

    try {
      auto check_time = Time::now();

      switch(state) {
        /* Detect markers in entire timage and estimate pose */
        case 0:
          aruco::detectMarkers(image, dictionary, corners, ids,
                              detectorParams, rejected);

          if(use_boxes) {
            next_state = 1;
          }
          break;

        /* Use bounding boxes to update robot poses */
        case 1:
          /* Prepare futures in case we're using threading */
          std::vector<std::future<std::pair<int, std::vector<Point2f>>>> futures;

          if(robot_poses.empty()) {
            next_state = 0;
          }

          for(auto it = robot_poses.begin(); it != robot_poses.end(); it++) {
            /* Make sure that we don't include reference markers in IDs */
            if(find(reference_marker_ids.begin(), reference_marker_ids.end(), it->first) != reference_marker_ids.end()) {
              continue;
            }

            auto f = [use_single_dictionaries, &current_time, &imageCopy,
                      &single_dictionaries, &dictionary,
                      &detectorParams, &next_state, &image, &H_inv, it] {

              auto time_now = Time::now();
              std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - current_time);

              /* Update bounding box coordinates based on previously recorded position */
              double prev_x = it->second.x;
              double prev_y = it->second.y;
              double change = time_span.count()*0.1 + 0.04;
              double x_max = prev_x + change;
              double x_min = prev_x - change;
              double y_max = prev_y + change;
              double y_min = prev_y - change;

              /* Vectors of points for the bounding boxes in local and homogeneous coordinates */
              std::vector<Point2f> rectangle_points;
              std::vector<Point3f> rectangle_points_h;

              /* Store the corners of the rectangles (plus the center for convenience) */
              rectangle_points.push_back(Point2f(x_min, y_max)); // Upper left
              rectangle_points.push_back(Point2f(x_min, y_min)); // Lower left
              rectangle_points.push_back(Point2f(x_max, y_min)); // Lower right
              rectangle_points.push_back(Point2f(x_max, y_max)); // Upper right
              rectangle_points.push_back(Point2f(prev_x, prev_y)); // Center

              convertPointsToHomogeneous(rectangle_points, rectangle_points_h);

              /* Use the reverse homography to map from homogeneous world to homogeneous image coordinates */
              cv::transform(rectangle_points_h, rectangle_points_h, H_inv);
              /* Clear this to avoid bad results form reusing the variable */
              rectangle_points.clear();
              convertPointsFromHomogeneous(rectangle_points_h, rectangle_points);

              /* Holds clamped x and y values from image coordinate.  The bounding box is formed from
              these values shortly */
              std::vector<double> xs;
              std::vector<double> ys;

              /* Clamp x elements to frame */
              std::transform(rectangle_points.begin(), rectangle_points.end(), std::back_inserter(xs),
              [](const cv::Point2f& point) {
                return std::min(std::forward<double>(std::max(point.x, 0.0f)), (double) frameWidth);
              });

              /* Clamp y elements to frame */
              std::transform(rectangle_points.begin(), rectangle_points.end(), std::back_inserter(ys),
              [](const cv::Point2f& point) {
                return std::min(std::forward<double>(std::max(point.y, 0.0f)), (double) frameHeight);
              });

              /* Cast to int because we're using pixles */
              int max_x = *std::max_element(xs.begin(), xs.end());
              int max_y = *std::max_element(ys.begin(), ys.end());
              int min_x = *std::min_element(xs.begin(), xs.end());
              int min_y = *std::min_element(ys.begin(), ys.end());

              /* Draw rectangle around the area in which we're looking */
              rectangle(imageCopy, Point2f(min_x, min_y), Point2f(max_x, max_y),  Scalar(0, 255, 255));

              /* Detect markers and estimate pose in the bounding box */
              cv::Mat local_image = image(cv::Rect(min_x, min_y, max_x-min_x, max_y-min_y));

              /* Prepare variables for detection in bounding box coordinates */
              vector<int> local_ids;
              vector<vector<Point2f>> local_corners, local_rejected;

              /* Whether to use single dictionaries */
              if(use_single_dictionaries) {
                /* Look for just a single marker in the bounding box */
                aruco::detectMarkers(local_image, single_dictionaries[it->first], local_corners, local_ids,
                                     detectorParams, local_rejected);
              } else {
                /* Look for all the markers in the bounding box */
                aruco::detectMarkers(local_image, dictionary, local_corners, local_ids,
                                     detectorParams, local_rejected);
              }

              /* If we didn't find anything locally */
              if(local_ids.size() == 0) {
                /* Go back to state 0 to search for markers */
                std::vector<Point2f> ret;
                next_state = 0;

                /* Return a -1 as an ID so that we know something weird happened */
                return std::make_pair(-1, ret);
              } else {

                /* Shift bounding box coordinates to world coordinates */
                local_corners[0][0].x += min_x;
                local_corners[0][0].y += min_y;
                local_corners[0][1].x += min_x;
                local_corners[0][1].y += min_y;
                local_corners[0][2].x += min_x;
                local_corners[0][2].y += min_y;
                local_corners[0][3].x += min_x;
                local_corners[0][3].y += min_y;

                /* If using single eMake sure to add on the ID because the
                local_id will be 0 */
                int local_id = use_single_dictionaries ? it->first : 0;
                return std::make_pair(local_ids[0]+local_id, local_corners[0]);
              }
            };

            /* If we're using threading, spin up the detection in another thread */
            if(async_detection) {
              futures.push_back(std::async(std::launch::async, f));
            } else {

              auto result = f();

              /* ID will be -1 if it wasn't found */
              if(result.first > 0) {

                /* Push back the marker that we found */
                ids.push_back(result.first);
                corners.push_back(result.second);
              } else {
                /* We didn't find one of the previously tracked IDs, so go back to
                the first state we can clear these variables because no threads
                are using them in the background. */

                robot_poses.clear();
                single_dictionaries.clear();
                next_state = 0;
              }
            }
          }

          /* If async is enabled, get results from async futures that we spun up earlier */
          bool clear_data = false;
          if(async_detection) {
            for(int i = 0; i < futures.size(); ++i) {
              auto result = futures[i].get();

              /* ID will be -1 if it wasn't found */
              if(result.first > 0) {

                ids.push_back(result.first);
                corners.push_back(result.second);
              } else {
                /* We didn't find one of the previously tracked IDs, so go back to
                 the first state and set boolean to clear data */

                 clear_data = true;
                 next_state = 0;
              }
            }

            /* Wait until the end to clear data to not corrupt any running
            threads */
            if(clear_data) {

              robot_poses.clear();
              single_dictionaries.clear();
            }
          }

          break;
      }

      /* If we foudn any IDs, render them on the screen */
      if(ids.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
      }

      /* Update and display detection time */
      auto check_time_end = Time::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(check_time_end - check_time);
      ss.str("");
      ss.clear();
      ss  << std::fixed<< std::setprecision(3) << time_span.count()*1000.0f;
      const String time_str = "elapsed time: " + ss.str();
      putText(imageCopy, time_str, Point(20, 20), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));

      if(ids.size() > 0) {
        /* Create JSON message to eventually send over MQTT */
        json message = {};

        /* Calculate robot poses and populate JSON message with data */
        for(unsigned int i = 0; i < ids.size(); i++) {
          /* Variables to hold homogeneous and eventual world (in meters) coordinates */
          std::vector<cv::Point3f> homogeneous_points;
          std::vector<Point2f> world_points;

          /* Use homography to convert image to world coordinates (in meters) */
          convertPointsToHomogeneous(corners[i], homogeneous_points);
          cv::transform(homogeneous_points, homogeneous_points, H);
          convertPointsFromHomogeneous(homogeneous_points, world_points);

          /* Find the orientation of the robot from the forward vector */
          Point2f forward_vector = (world_points[1]+world_points[2]-world_points[0]-world_points[3])/2;
          double orientation = atan2(forward_vector.y, forward_vector.x);

          /* Find the (x, y) pose of the robot by taking the average of the 4 marker corners */
          double x = 0.25*(world_points[0].x + world_points[1].x + world_points[2].x + world_points[3].x);
          double y = 0.25*(world_points[0].y + world_points[1].y + world_points[2].y + world_points[3].y);

          /* Convert ID to string for use in JSON message */
          std::string id = std::to_string(ids[i]);

          // Make sure that we don't include reference markers in IDs
          if(find(reference_marker_ids.begin(), reference_marker_ids.end(), ids[i]) != reference_marker_ids.end()) {
            continue;
          }

          /* Only store new dictionaries if we're looking for markers */
          if(state == 0) {
            single_dictionaries[ids[i]] = generate_single_dictionary(ids[i], 4, dictionary);
          }

          /* Update the previous robot pose for use with the bounding boxes */
          robot_poses[ids[i]] = Point2f(x, y);

          /* Add pose to msg */
          message[id]["x"] = x;
          message[id]["y"] = y;
          message[id]["theta"] = orientation;

          /* Add total detection time to message */
          message[id]["total_time"] = time_span.count();

          /* Add battery voltage data if available, or -1 otherwise */
          if(powerData.find(id) != powerData.end()) {
            message[id]["powerData"] = powerData[id];
          } else {
            message[id]["powerData"] = -1;
          }

          /* Add charging status data if available, or -1 otherwise */
          if(status_data.find(id) != status_data.end()) {
            message[id]["charging"] = status_data[id];
          } else {
            message[id]["charging"] = -1;
          }

          // double batteryLevel = (double) message[id]["powerData"];
    			// ss.str("");
    			// ss.clear();
    			// ss  << std::fixed<< std::setprecision(3) << batteryLevel;
          //       const String powerDataStr = "battery: " + ss.str();
    			// ss.str("");
    			// ss.clear();
    			// ss  << std::fixed<< std::setprecision(3) << static_cast<double>(message[id]["x"]);
          //       const String xStr = "x:       " + ss.str();
    			// ss.str("");
    			// ss.clear();
    			// ss  << std::fixed<< std::setprecision(3) << static_cast<double>(message[id]["y"]);
          //       const String yStr = "y:       " + ss.str();
    			// ss.str("");
    			// ss.clear();
    			// ss  << std::fixed<< std::setprecision(3) << static_cast<double>(message[id]["theta"]);
          //       const String thetaStr = "theta:   " + ss.str();
          //       vector<double> rip;
          //       float u, v;
          //       u = 0.25*(corners[i][0].x+corners[i][1].x+corners[i][2].x+corners[i][3].x);
          //       v = 0.25*(corners[i][0].y+corners[i][1].y+corners[i][2].y+corners[i][3].y);
          //       rip.push_back(u);
          //       rip.push_back(v);
          //       robotImagePosition[id] = rip;
          //
          // if (clickedOnGritsbot[id]) {
          //     putText(imageCopy, powerDataStr, Point(u + 20, v - 20), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));
          //     putText(imageCopy, xStr, Point(u + 20, v + -5), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));
          //     putText(imageCopy, yStr, Point(u + 20, v + 10), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));
          //     putText(imageCopy, thetaStr, Point(u + 20, v + 25), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));
          // } else if (closeToGritsbot[id]) {
          //     putText(imageCopy, powerDataStr, Point(u + 20, v - 20), CV_FONT_NORMAL, 0.5, Scalar(0, 180, 180));
          //     putText(imageCopy, xStr, Point(u + 20, v + -5), CV_FONT_NORMAL, 0.5, Scalar(0, 180, 180));
          //     putText(imageCopy, yStr, Point(u + 20, v + 10), CV_FONT_NORMAL, 0.5, Scalar(0, 180, 180));
          //     putText(imageCopy, thetaStr, Point(u + 20, v + 25), CV_FONT_NORMAL, 0.5, Scalar(0, 180, 180));
					// }
        }

        /* Send MQTT message as JSON dump */
        std::string s = message.dump();

        /* Publish MQTT message */
        if(parser.has("h")) {
          m->async_publish(main_publish_channel, s);
        }
      }

      /* Section to display and update loop time */
      auto time_now = Time::now();
      std::chrono::duration<double> time_span_loop = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - current_time);
      ss.str("");
      ss.clear();
      ss  << std::fixed << std::setprecision(3) << time_span_loop.count()*1000.0f;
      const String loop_time_str = "elapsed time loop: " + ss.str();
      putText(imageCopy, loop_time_str, Point(20, 40), CV_FONT_NORMAL, 0.5, Scalar(0, 255, 255));

      current_time = time_now;

      /* Functionality for drawing rejected markers on output image */
      if(showRejected && rejected.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, rejected, noArray(),
                                    Scalar(100, 0, 255));
      }

      /* Show rendered image on screen */
      imshow("out", imageCopy);

      /* If escapse is pressed, exit the loop */
      char key = (char) waitKey(1);
      if(key == 27) {
        break;
      }

      /* If we're pressing space detect all markers */
      if(key == 32) {
        single_dictionaries.clear();
        robot_poses.clear();
        next_state = 0;
      }

      /* Go to the chosen next state at the end of the main loop */
      state = next_state;

    } catch (int e) {
      std::cout << "Encountered an exception" << std::endl;
    }
  }

  /* Close capture device */
  if(inputVideo.isOpened()) {
    inputVideo.release();
  }

  return 0;
}
