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

#include <opencv2/highgui.hpp>
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

/* Include Boost library */
#include <boost/filesystem.hpp>

#include <json.hpp>
#include <iostream>
#include <fstream>
#include <mqttclient.hpp>

/* Chrono typedefs */
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

/* Camera calibration constants */
double OFFSET_X_PX = -30;
double OFFSET_Y_PX = -9;

double SCALE_X = 1.18 / 979.0;
double SCALE_Y = 0.78 / 647.0;
double ROTATION = (0.75 * M_PI/180.0);

/* Convenience declaration */
using json = nlohmann::json;

/* Name spaces */
using namespace std;
using namespace cv;

/* Forward declarations */
const std::string currentDateTime();
bool  openOutputVideo(std::string filename, int codec, int fps);
bool  createDirectory(std::string folderName);
bool  getJSONString(auto data, std::string fieldName, std::string* output);
bool  getJSONInt(auto data, std::string fieldName, int* output);

/* Global variables */
json powerData;

/* Tracker parameters */
int   dictionaryId;
bool  showRejected;
bool  estimatePose;
float markerLength;
bool  useMetric;
int   camId;
Mat   camMatrix, distCoeffs;

/* OpenCV video input and output devices */
VideoCapture  inputVideo;
VideoWriter   outputVideo;

/* Video recording flags */
bool        recordVideo     = false;
bool        recordFrames    = false;
int         recordVideoFPS  = 30;
int         recordImageFPS  = 1;
uint16_t    frameNumber     = 0;
std::string imageFolderName = "./frames";
bool        outputVideoWithMarkers = false;

/* Codec settings for MPEG encoded video files */
//string  codec       = "MPG";
//int     videoCodec  = CV_FOURCC('M','P','E','G');

/* Codec settings for X264 encoded MP4 files */
string  codec       = "MP4";
int videoCodec = 0x21;

/* Time stamps */
auto    lastVideoFrameWrite = Time::now();
auto    lastImageFrameWrite = Time::now();
double  totalTime = 0.0;
int     totalIterations = 0;
double  tick = (double) getTickCount();

/* Framesize */
int       frameWidth  = 1280;
int       frameHeight = 720;
cv::Size  frameSize   = cv::Size(frameWidth, frameHeight);

/* MQTT parameters */
double      publishRate   = 30;
bool        configUpdate  = false;
std::string statusMsg     = "";

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
  } catch (const std::exception& e) {

  }
}

void configCallback(std::string topic, std::string message) {
  /* Parameters in JSON message for video recording
   * type       : video       [string]
   * run        : start/stop  [string]
   * frameRate  : 15, 30      [int]
   * folderName :             [string]
   * fileName   :             [string]
   *
   * Parameters in JSON message for image recording
   * type       : image       [string]
   * run        : start/stop  [string]
   * frameRate  : N           [int]
   * folderName :             [string]
   *
   */

  /* Initialize status message and expected parameters */
  std::string statusMsg = "";
  std::string type = "";
  std::string run = "";
  std::string folderName = "";
  std::string fileName = "";

  std::cout << message << std::endl;

  /* Parse message string into JSON dictionary */
  try {
	auto data = json::parse(message);

    if(getJSONString(data, std::string("type"), &type)) {
      /* Debug output */
      std::cout << "Config Message: " << message << std::endl;
      std::cout << "Tracker config type: " << type << std::endl;

      if(type.compare("video") == 0) {
        /* Video is requested to be recorded */
        if(getJSONString(data, std::string("run"), &run)) {
          if(run.compare("start") == 0) {
            // 1. if run == start
            // 1.a: create output folder
            // 1.b: open output stream
            // 1.c: set global recording parameters

            std::cout << "Run Video: " << run << std::endl;

            /* Start recording the video after reading parameters */
            if(getJSONString(data, std::string("folderName"), &folderName) &&
                getJSONString(data, std::string("fileName"), &fileName) &&
                getJSONInt(data, std::string("frameRate"), &recordVideoFPS)) {

              /* Debug output */
              std::cout << "folderName: " << folderName << std::endl;
              std::cout << "fileName: " << fileName << std::endl;
              std::cout << "frameRate: " << recordVideoFPS << std::endl;

              /* Create output folder */
              if(createDirectory(folderName)) {
                /* Open parameterized output stream */
                openOutputVideo(folderName + "/" + fileName, videoCodec, recordVideoFPS);

                /* Start recording video */
                if(outputVideo.isOpened()) {
                  recordVideo = true;
                  std::cout << "Video file opened: " << folderName + "/" + fileName << std::endl;
                } else {
                  std::cout << "Video file opened: " << folderName + "/" + fileName << " failed to open!!!!" << std::endl;
                }
              } else {
                std::cout << "Folder creation failed: " << folderName << std::endl;
              }
            }
          } else {
            // 2. if run == stop
            // 2.a: stop output stream
            // 2.b: set recording flags to false
            /* Stop recording video */
            recordVideo = false;
            totalTime = 0.0;

            std::cout << "Run Video level stop detected." << std::endl;

            if(outputVideo.isOpened()) {
              //std::cout << "Stopping video recording" << std::endl;
              //outputVideo.release();
              std::cout << "Stopped video recording" << std::endl;
            }
          }
        }
      } else if(type.compare("image") == 0) {
        // 1. if run == start
        // 1.a: create output folder
        // 1.c: set global recording parameters
        // 2. if run == stop
        // 2.a: set recording flags to false

        //* type       : image       [string]
        //* run        : start/stop  [string]
        //* frameRate  : N           [int]
        //* folderName :             [string]

        if(getJSONString(data, std::string("run"), &run)) {
          /* Debug output */
          std::cout << "Run Image: " << run << std::endl;

          if(run.compare("start") == 0) {
            /* Start recording images after reading parameters */
            if(getJSONString(data, std::string("folderName"), &folderName) &&
                getJSONInt(data, std::string("frameRate"), &recordImageFPS)) {

              /* Debug output */
              std::cout << "folderName: " << folderName << std::endl;
              std::cout << "frameRate: " << recordImageFPS << std::endl;

              if(createDirectory(folderName)) {
                frameNumber   = 0;
                recordFrames  = true;
                imageFolderName = folderName;
              }
            } else {
              recordFrames = false;
            }
          } else {
            recordFrames = false;
            std::cout << "Run Image stop detected: " << run << std::endl;
          }
        }
      } else {
        statusMsg = "Error: Type " + type + " is not a valid option.";
      }
    }
  } catch (const std::exception& e) {
	std::cout << "Exception caught" << std::endl;
  }
  configUpdate = true;
}

/* Cast function to standard function pointer */
std::function<void(std::string, std::string)> stdf_powerDataCallback = &powerDataCallback;
std::function<void(std::string, std::string)> stdf_configCallback = &configCallback;

/* -------------------------------------
 *		  JSON Utility functions
 * ------------------------------------- */
bool getJSONString(auto data, std::string fieldName, std::string* output) {
  if (data.find(fieldName) != data.end()) {
    if(data[fieldName].is_string()) {
      *output = data[fieldName];
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool getJSONInt(auto data, std::string fieldName, int* output) {
  if (data.find(fieldName) != data.end()) {
    if(data[fieldName].is_number()) {
      *output = data[fieldName];
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* -------------------------------------
 *		Video and frame output functions
 * ------------------------------------- */
void writeFrameToVideo(Mat frame) {
  if( (Time::now() - lastVideoFrameWrite).count() > (1.0 / recordVideoFPS) ) {
    /* Update time stamps of last frame */
    lastVideoFrameWrite = Time::now();

    /* Debug output */
    //totalTime += (Time::now() - lastVideoFrameWrite).count();
    //std::cout << totalTime << std::endl;

    /* Add frame to output video */
    if(outputVideo.isOpened()) {
      outputVideo << frame;
    }
  }
}

bool writeFrameToFile(Mat frame, std::string filename) {
  if( (Time::now() - lastImageFrameWrite).count() > (1.0 / recordImageFPS) ) {
    /* Update time stamps of last frame */
    lastImageFrameWrite = Time::now();

    /* Write file */
    imwrite(filename, frame);

    /* Debug output */
    //std::cout << "Printing to file: " << filename << std::endl;

    return true;
  } else {
    return false;
  }
}

bool openOutputVideo(std::string filename, int codec, int fps) {
  /* Opening succeeds if the filename is .mpeg only */
  /* Available codecs according to http://www.fourcc.org/codecs.php
    * - X264
    * - XVID
    * - MPG4
    * - MJPG
    */
  if(outputVideo.open(filename, videoCodec, fps, frameSize, true)) {
    if(outputVideo.isOpened()) {
      std::cout << "Recording video with filename " << filename << std::endl;
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

std::string getImageFilename(std::string folder, int frameNumber) {
  std::string filenameInit = folder + "/00001.jpg";
  char filenameArray[sizeof(char) * filenameInit.length()];
  sprintf(filenameArray, "%s/%05d.jpg", folder.c_str(), frameNumber);

  return std::string(filenameArray);
}

bool createDirectory(std::string folderName) {
  /* Create directory in boost format */
  boost::filesystem::path dir(folderName);

  /* Check if directory already exists */
  if (boost::filesystem::is_directory(dir) &&
      boost::filesystem::exists(dir)) {
    return true;
  }

  /* Create actual directory */
  if (boost::filesystem::create_directory(dir)) {
    if (boost::filesystem::is_directory(dir) &&
          boost::filesystem::exists(dir)) {
      /* Directory exists */
      std:string dirStr = boost::filesystem::canonical(dir).string();
      std::cout << "Boost folder created: " << dirStr << std::endl;
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* -------------------------------------
 *		Calibration utility functions
 * ------------------------------------- */
double rotate_x(double x, double y) {
	return cos(ROTATION)*x - sin(ROTATION)*y;
}

double rotate_y(double x, double y) {
	return sin(ROTATION)*x + cos(ROTATION)*y;
}

/* -------------------------------------
 *		Time utility functions
 * ------------------------------------- */
/* Get current date/time, format is YYYY-MM-DD.HH:mm:ss */
const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);

  /* Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
   * for more information about date/time format
   * strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
   */
  strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tstruct);

  return buf;
}

void printDetectionRate() {
  double currentTime = (double) (getTickCount() - tick) / getTickFrequency();
  totalTime += currentTime;
  totalIterations++;

  if(totalIterations % 30 == 0) {
    std::cout << "Detection Time = " << currentTime * 1000 << " ms "
              << "(Mean = " << 1000 * totalTime / double(totalIterations)
              << " ms)" << std::endl;
  }

  tick = (double) getTickCount();
}

/* --------------------------------------------------
 *		Camera/tracker configuration functions
 * -------------------------------------------------- */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
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
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
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
	        "{v        |       | Input from video file, if ommited, input comes from camera }"
	        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
	        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
	        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
	        "{dp       |       | File of marker detector parameters }"
	        "{r        |       | show rejected candidates too }"
	        "{vo       |       | Record video to output file }"
	        "{rM       |       | Render detected markers into output video file }"
	        "{m        |       | Use metric units in position updates of MQTT messages }"
			    "{od       |       | Output file for video }"
			    "{mqtt     |       |  MQTT setup information}"
			    "{h        | localhost | MQTT broker }"
			    "{p        | 1883      | MQTT port }";
}

/* --------------------------------------
 *		        MAIN FUNCTION
 * -------------------------------------- */
int main(int argc, char *argv[]) {
	/* Initialize command line parsing */
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);

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
	estimatePose = parser.has("c");
	markerLength = parser.get<float>("l");
	useMetric = parser.has("m");
	camId = parser.get<int>("ci");

	/* Initialize detector parameters from file */
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	if(parser.has("dp")) {
		if(!readDetectorParameters(parser.get<string>("dp"), detectorParams)) {
			cerr << "Invalid detector parameters file" << endl;
			return 0;
		}
	}

	/* Add corner refinement in markers */
	detectorParams->cornerRefinementMethod = 0;

	/* Instantiate dictionary containing tags */
	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	if(estimatePose) {
		if(!readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs)) {
			cerr << "Invalid camera file" << endl;
			return 0;
		}
	}

	/* Read video/image recording command line parameters */
	// TODO: remove or replace --> this is only used as file name for
	//				input videos (i.e. non-live tracking)
	//				--> not useful for our purposes, we only do live camera
	//				stream tracking
	recordVideo = parser.has("vo");
	outputVideoWithMarkers = parser.has("rM");

	/* Open camera input stream */
	inputVideo.open(camId);

  /* Set resolution to 1280 x 720 */
	inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, frameWidth);
	inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight);

	std::cout << "Image size: " << frameSize.width
						<< " / " << frameSize.height << std::endl;

	/* Initialize time stamps */
  lastVideoFrameWrite = Time::now();
  lastImageFrameWrite = Time::now();

  /* Open video writer if video output filename is specified */
  if(recordVideo) {
    /* Create time stamped filename */
    std::transform(codec.begin(), codec.end(),codec.begin(), ::tolower);
    string videoFilename = "video_" + currentDateTime() + "." + codec;

    /* Open video file */
    openOutputVideo(videoFilename, videoCodec, recordVideoFPS);
  }

	/* Set up MQTT client */
	MQTTClient m(parser.get<string>("h"), parser.get<int>("p"));
	m.start();

	/* Set MQTT publishing topic */
	std::string main_publish_channel  = "overhead_tracker/all_robot_pose_data";
  std::string topicAck              = "overhead_tracker/config_ack";

	/* Subscribe to power data channels */
	for (int index = 0; index < 50; index++) {
		/* Create topic name string */
		std::string topicName = std::to_string(index) + "/power_data";

		/* Subscribe to MQTT channel */
		m.subscribe(topicName, stdf_powerDataCallback);
		//std::cout << "Subscribed to topic: " << topicName  << std::endl;
	}

	/* Subscribe to configuration channel */
	m.subscribe("overhead_tracker/config", stdf_configCallback);

  /* --------------------------------------------
   *                Main event loop
   * -------------------------------------------- */
  while(inputVideo.grab()) {
    /* Retrieve new frame from camera */
    Mat image, imageCopy;
    inputVideo.retrieve(image);

    vector<int> ids;
    vector< vector<Point2f>> corners, rejected;
    vector<Vec3d> rvecs, tvecs;
    Mat rotMat;

    try {
      /* Detect markers and estimate pose */
      aruco::detectMarkers(image, dictionary, corners, ids,
                            detectorParams, rejected);
      if(estimatePose && ids.size() > 0) {
        aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix,
                                          distCoeffs, rvecs, tvecs);
      }

      /* Render results */
      image.copyTo(imageCopy);
      if(ids.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        if(estimatePose) {
          for(unsigned int i = 0; i < ids.size(); i++) {
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs,
                          rvecs[i], tvecs[i], markerLength * 0.5f);
          }
        }
      }

      /* Write frame to video at specified fps if appropriate flags are set */
      if(recordVideo && outputVideo.isOpened()) {
        if(outputVideoWithMarkers) {
          /* Render coordinate systems and markers into video frame */
          writeFrameToVideo(imageCopy);
        } else {
          /* Render video without any markers or coordinate systems */
          writeFrameToVideo(image);
        }
      }

      if(recordFrames) {
        /* Set file name */
        std::string filename = getImageFilename(imageFolderName, frameNumber);

        /* Write image to jpg file */
        if(writeFrameToFile(image, filename)) {
          /* Update frame number */
          frameNumber++;
        }
      }

      /* Create and send MQTT message */
      if(ids.size() > 0) {
        if(estimatePose) {
          /* Create MQTT message */
          json message = {};

          /* Populate msg with data */
          for(unsigned int i = 0; i < ids.size(); i++) {
            std::string id = std::to_string(ids[i]);

            /* Declare variables used for position and angle computation */
            Point2f dP(0, 0);
            Point2f xM(0, 0);
            vector< Point2f > imgPoints;
            vector< Point3f > inputPoints;

            /* Add image coordinates to message as the center
              * of marker coordinates */
            Mat currentMarker = static_cast<InputArrayOfArrays>(corners).getMat(i);
            Point2f cent(0, 0);
            for(int p = 0; p < 4; p++) {
              cent += currentMarker.ptr< Point2f >(0)[p];
            }
            cent = cent / 4.;

            message[id]["u"] = cent.x;
            message[id]["v"] = cent.y;

            /* Add battery voltage data if available, or -1 otherwise */
            if(powerData.find(id) != powerData.end()) {
              message[id]["powerData"] = powerData[id];
            } else {
              message[id]["powerData"] = -1;
            }

            /* Publish metric or image coordinates based on input flag -m */
            if(useMetric) {
              /* Project world coordinates onto image plane to
                * compute angle of x-axis, i.e.
                *
                *  1. project point (0,0,0) into image plane --> o_c
                *  2. project point (1,0,0) into image plane --> x_M
                *  3. compute angle between x_M and x_c
                * */
              inputPoints.push_back(Point3f(0,0,0));
              inputPoints.push_back(Point3f(0.1,0,0));

              /* Project points into image plane */
              // Syntax: cv2::projectPoints(axisPoints, _rvec, _tvec,
              //              _cameraMatrix, _distCoeffs, imagePoints);
              projectPoints(inputPoints, rvecs[i], tvecs[i],
                            camMatrix, distCoeffs, imgPoints);
              Point2f xM = imgPoints[1] - imgPoints[0];

              /* Add pose to msg */
              message[id]["x"] = tvecs[i][0];
              /* Flip y-axis to make the coordinate system right-handed */
              message[id]["y"] = -tvecs[i][1];
              // Add rotatioin to message
              message[id]["theta"] = atan2(-xM.y, xM.x);
            } else {
              /* Compute orientation of marker through corners of
                * marker in image coordinates
                */
              Point2f dP = currentMarker.ptr< Point2f >(0)[1] - currentMarker.ptr< Point2f >(0)[0];
              Point2f dP2 = currentMarker.ptr< Point2f >(0)[2] - currentMarker.ptr< Point2f >(0)[3];

              /* Add pose to msg (i.e. center of marker)
                *  Center coordinate systems in the center of the image
                *  such that x \in [-640, 640] and y \in [-360, 360].
                *  Additionally, flip the y-axis to make the coordinate
                *  system right-handed.
                */

              double x = (cent.x - frameSize.width/2 - OFFSET_X_PX)  * SCALE_X;
              double y = (frameSize.height/2 - cent.y - OFFSET_Y_PX) * SCALE_Y;
              double x_temp = rotate_x(x, y);
              y = rotate_y(x, y);
              x = x_temp;

              /* Add coordinates to MQTT message */
              message[id]["x"] = x;
              message[id]["y"] = y;

              // We compute theta using the average of the corner points
              message[id]["theta"] = atan2(-(dP.y + dP2.y)/2.0, (dP.x + dP2.x)/2.0);
            }
          }

          /* Send MQTT message as JSON dump */
          std::string s = message.dump();
          //std::cout << s << std::endl;

          /* Publish MQTT message */
          m.async_publish(main_publish_channel, s);
        }
      }

      if(showRejected && rejected.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, rejected, noArray(),
                                    Scalar(100, 0, 255));
      }

      /* Show rendered image on screen */
      imshow("out", imageCopy);

      char key = (char)waitKey(1);
      if(key == 27) {
        std::cout << "Escape pressed" << std::endl;
        break;
      }
    } catch (int e) {
      // TODO: Exception handling
      std::cout << e << std::endl;
    }

    /* Debug output if desired */
    //printDetectionRate();

    /* Send status ack message if configuration changes */
    if(configUpdate) {
      json statusMessage = {};
      statusMessage["status"] = statusMsg;
      m.async_publish(topicAck, statusMessage.dump());

      configUpdate = false;
    }
  }

  /* Close capture device */
  if(inputVideo.isOpened()) {
    inputVideo.release();
  }

  return 0;
}
