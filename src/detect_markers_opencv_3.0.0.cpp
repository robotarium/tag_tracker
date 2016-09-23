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

/* For Rodrigues transformation */
#include <opencv2/calib3d/calib3d.hpp>

#include <json.hpp>
#include <iostream>
#include <fstream>
#include <mqttclient.hpp>

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

/* Global variables */
json powerData;

VideoCapture inputVideo;
VideoWriter outputVideo;

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

/* Cast function to standard function pointer */
std::function<void(std::string, std::string)> stdf_powerDataCallback = &powerDataCallback;

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

const double timeDelta(timespec tPast, clockid_t id) {
  /* NOTE: For a description of clockid_t see
   *  http://nadeausoftware.com/articles/2012/04/c_c_tip_how_measure_elapsed_real_time_benchmarking
   *  http://linux.die.net/man/3/clock_gettime
   * NOTE: This function is based on the following post
   *  http://stackoverflow.com/questions/7472071/is-gettimeofday-guaranteed-to-be-of-microsecond-resolution
   */
  double dt;
  struct timespec tNow;

  clock_gettime(id, &tNow);
  dt = tNow.tv_sec - tPast.tv_sec + (tNow.tv_nsec - tPast.tv_nsec) / 1e9;

  return dt;
}

static inline timespec timeAdd(timespec oldTime, timespec time) {
  /* For a description of time arithmetic functions see
   * http://codereview.stackexchange.com/questions/40176/correctness-of-calculations-with-struct-timespec
   */
  if (time.tv_nsec + oldTime.tv_nsec >= 1E9) {
    return (timespec){
      tv_sec: time.tv_sec + oldTime.tv_sec + 1,
      tv_nsec: time.tv_nsec + oldTime.tv_nsec - (long int) 1E9 // Cast to (long int) to avoid complier warning
    };
  } else {
    return (timespec){
      tv_sec: time.tv_sec + oldTime.tv_sec,
      tv_nsec: time.tv_nsec + oldTime.tv_nsec
    };
  }
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
    fs["doCornerRefinement"] >> params->doCornerRefinement;
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
			//"{a        |       | Tag aliases }"
			"{mqtt        |       |  MQTT setup information}"
			 "{h        | localhost      | MQTT broker }"
			"{p        | 1884      | MQTT port }";
}

/**
	TODO: GRAB JSON CONFIG FILE FROM COMMAND LINE AND PUBLISH ROBOT ALIASES WITH DATA RATHER THAN TAG IDENTITIES
 */
int main(int argc, char *argv[]) {
	/* Command line parsing */
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");
    bool useMetric = parser.has("m");

    cout << "Use metric coordinates: " << useMetric << endl;

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    /* Video recording flags */
    bool recordVideo = parser.has("vo");
    bool recordFrames = false;
    bool recordFramesFPS = 1;
    bool outputVideoWithMarkers = parser.has("rM");
	uint16_t frameNumber = 0;

    Size frameSize = Size((int) 1280, (int) 720);
    int waitTime;

    if(!video.empty()) {
      inputVideo.open(video);
      waitTime = 0;
    } else {
      inputVideo.open(camId);
      waitTime = 1;

      /* dpickem: Set resolution to 1280 x 720 */
      inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
      inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

      //Size frameSize = Size((int) inputVideo.get(CV_CAP_PROP_FRAME_WIDTH),
      //                 (int) inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
      cout << "Image size: " << frameSize.width << " / " << frameSize.height << endl;
    }

    /* Open video writer if video output filename is specified */

    double fps = 30;
    double publishRate = 30;

    timespec lastFrameWrite;
    timespec timestepFrame;
    timespec lastWebsocketPublish;
    const clockid_t clockId = CLOCK_MONOTONIC;
    clock_gettime(clockId, &lastFrameWrite);
    clock_gettime(clockId, &lastWebsocketPublish);

    string codec = "MPG";
    //int videoCodec = CV_FOURCC(codec.at(0),codec.at(1),codec.at(2),codec.at(3));
    int videoCodec = CV_FOURCC('M','P','E','G');

    if(recordVideo) {
      /* Create time stamped filename */
      std::transform(codec.begin(), codec.end(),codec.begin(), ::tolower);
      string videoFilename = "video_" + currentDateTime() + "." + codec;
			if(parser.has("od")) {
				//If there's an output file, append the desired directory info
				videoFilename = parser.get<String>("od") + videoFilename;
			}

      /* Opening succeeds if the filename is .mpeg only */
      /* Available codecs according to http://www.fourcc.org/codecs.php
       * - X264
       * - XVID
       * - MPG4
       * - MJPG
       */
      if(outputVideo.open(videoFilename, videoCodec, fps, frameSize, true)) {
        cout << "Recording video with filename " << videoFilename << endl;
        cout << "RecordVideo / isOpened / withMarkers: " << recordVideo << " / " << outputVideo.isOpened() << " / " << outputVideoWithMarkers << endl;
      }
    }

		// READ IN ALIASES FROM FILE


		// json aliases;
		//
		// if(parser.has("a")) {
		//
		// 	string input_filename = parser.get<string>("a");
		// 	std::cout << "Received alias file @ " << input_filename << std::endl;
		//
		// 	std::ifstream file;
		// 	file.open(input_filename, std::ifstream::in);
		// 	file >> aliases;
		// 	file.close();
		// }

		// json mqtt_setup;
		//
		// if(parser.has("mqtt")) {
		//
		// 	string input_filename = parser.get<string>("mqtt");
		// 	std::cout << "Received mqtt file @ " << input_filename << std::endl;
		//
		// 	mqtt_setup = json::parse(input_filename);
		//
		// 	// std::ifstream file;
		// 	// file.open(input_filename, std::ifstream::in);
		// 	// file >> mqtt_setup;
		// 	// file.close();
		// }
		//
		// std::cout << mqtt_setup << std::endl;

		/* Set MQTT publishing topic */
		std::string main_publish_channel = "overhead_tracker/all_robot_pose_data"; //mqtt_setup["network_attributes"]["network_sub_attributes"]["all_robot_pose_data"];

		/* Set up MQTT client */
		MQTTClient m(parser.get<string>("h"), parser.get<int>("p"));
		m.start();

		/* Subscribe to power data channels */
		for (int index = 0; index < 50; index++) {
			/* Create topic name string */
			std::string topicName = std::to_string(index) + "/power_data";

			/* Subscribe to MQTT channel */
			m.subscribe(topicName, stdf_powerDataCallback);
			std::cout << "Subscribed to topic: " << topicName  << std::endl;
		}
		//m.subscribe("0/power_data", stdf_powerDataCallback);
		//std::cout << "Subscribed to topic: 0/power_data"  << std::endl;

    double totalTime = 0;
    int totalIterations = 0;

		double tick = (double) getTickCount();


    while(inputVideo.grab()) {

				Mat image, imageCopy;
        inputVideo.retrieve(image);

        vector<int> ids;
        vector< vector<Point2f>> corners, rejected;
        vector<Vec3d> rvecs, tvecs;
        Mat rotMat;

        try {
          /* Detect markers and estimate pose */
          aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
          if(estimatePose && ids.size() > 0) {
              aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
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

        /* Write frame to video at 15 fps if appropriate flags are set */
        if(recordVideo && outputVideo.isOpened()) {
          if(timeDelta(lastFrameWrite, clockId) > (1.0 / fps)) {
            /* Update time stamp of last frame */
            timestepFrame.tv_sec  = 0;
            timestepFrame.tv_nsec = (1.0 / fps) * 1E9;

            /* Set time stamp of last frame to old time stamp + 1 / fps sec */
            lastFrameWrite = timeAdd(lastFrameWrite, timestepFrame);

            if(outputVideoWithMarkers) {
              /* Render coordinate systems and markers into video frame */
              outputVideo << imageCopy;
            } else {
              /* Render video without any markers or coordinate systems */
              outputVideo << image;
            }
          }
        }

		if(recordFrames) {
          if(timeDelta(lastFrameWrite, clockId) > (1.0 / fps)) {
            /* Update time stamp of last frame */
            timestepFrame.tv_sec  = 0;
            timestepFrame.tv_nsec = (1.0 / recordFramesFPS) * 1E9;

            /* Set time stamp of last frame to old time stamp + 1 / fps sec */
            lastFrameWrite = timeAdd(lastFrameWrite, timestepFrame);

			/* Set file name */
			std::string filenameStr = "./frames/00001.jpg";
			char filename[sizeof(char) * filenameStr.length()];
			sprintf(filename, "./frames/%05d.jpg", frameNumber);
			filenameStr = std::string(filename);

			std::cout << "Printing to file: " << filenameStr << std::endl;

			/* Write file */
			imwrite(filename, image);
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

							//std::cout << "Got id: " << str_id << std::endl;

							// if(aliases[str_id] == NULL) {
							// 	//std::cout << "Didn't have alias for " << str_id << std::endl;
							// 	continue;
							// }

							//std::string id = std::to_string((int) aliases[str_id]);

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

							/* Add battery voltage data if available, if not set a default value of -1 */
							if(powerData.find(id) != powerData.end()) {
								//std::cout << "Power data in main event loop: " << powerData[id] << std::endl;
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

                message[id]["x"] = x;
                message[id]["y"] = y;

								// We compute theta using the average of the corner points
                message[id]["theta"] = atan2(-(dP.y + dP2.y)/2.0, (dP.x + dP2.x)/2.0);
              }
            }

            std::string s = message.dump();
			//std::cout << s << std::endl;

            //if(timeDelta(lastWebsocketPublish, clockId) > (1.0 / publishRate)) {
						m.async_publish(main_publish_channel, s);

              ///* Update time stamp of last frame */
              //timestepFrame.tv_sec  = 0;
              //timestepFrame.tv_nsec = (1.0 / publishRate) * 1E9;

              ///* Set time stamp of last frame to old time stamp + 1 / fps sec */
              //lastWebsocketPublish = timeAdd(lastWebsocketPublish, timestepFrame);
            //}
          }
        }

        if(showRejected && rejected.size() > 0) {
          aruco::drawDetectedMarkers(imageCopy, rejected, noArray(),
                                      Scalar(100, 0, 255));
        }

        imshow("out", imageCopy);

        char key = (char)waitKey(1);
        if(key == 27) break;
      } catch (int e) {

      }

			double currentTime = (double) (getTickCount() - tick) / getTickFrequency();
			totalTime += currentTime;
			totalIterations++;
			//if(totalIterations % 30 == 0) {
					//cout << "Detection Time = " << currentTime * 1000 << " ms "
							 //<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
			//}

			tick = (double) getTickCount();
    }

    /* Close capture device */
    if(inputVideo.isOpened()) {
      inputVideo.release();
    }

    return 0;
}
