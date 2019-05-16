/*pkg-config --libs --cflags opencv
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

using namespace std;
using namespace cv;

/*template<class T> using vector = std::vector<T>;
using cout = std::cout;
using cerr = std::cerr;
using endl = std::endl;*/

namespace {
const char* about = "Tag Tracker for the Robotarium!  Detects tags in a pre-defined space";
const char* keys  =
    "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
    "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
    "{dp       |       | File of marker detector parameters }"
    "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
    "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
    "{r        |       | show rejected candidates too }"
    "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1, CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}

/**
 */
bool readCameraParameters(const string& filename, Mat& camMatrix, Mat& distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) {
        return false;
    }

    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    return true;
}

/**
 */
bool readDetectorParameters(const string& filename, Ptr<aruco::DetectorParameters>& params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) {
        return false;
    }

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


/**
 */
int main(int argc, char *argv[]) {
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

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool ok = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!ok) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if (parser.has("refine")) {
        //override cornerRefinementMethod read from config file
        detectorParams->cornerRefinementMethod = parser.get<int>("refine");
    }
    std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    int camId = parser.get<int>("ci");

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool ok = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!ok) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    VideoCapture inputVideo;
    int waitTime = 1;
    inputVideo.open(camId);

    // THIS CODEC NEEDS TO BE CHANGED HERE FOR IT TO WORK
    int codec = VideoWriter::fourcc('M','J','P','G'); 
    inputVideo.set(CAP_PROP_FOURCC, codec);
    inputVideo.set(3, 1920);
    inputVideo.set(4, 1080);

    if(!inputVideo.isOpened()) {
        cerr << "Could not open video device" << endl;
        return -1;
    }

    double totalTime = 0;
    int totalIterations = 0;

    double tick = (double) getTickCount();
    auto start = chrono::steady_clock::now();
    Mat image;

    while(inputVideo.grab()) {
        inputVideo.retrieve(image);
                
        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;
        vector<Vec3d> rvecs, tvecs;

        tick = (double) getTickCount();
        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if(estimatePose && ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                             tvecs);

        /*double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }*/

        // draw results
        //image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(image, corners, ids);

            if(estimatePose) {
                for(size_t i = 0; i < ids.size(); ++i) {
                    aruco::drawAxis(image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(image, rejected, noArray(), Scalar(100, 0, 255));

        //currentTime = ((double) getTickCount() - tick) / getTickFrequency();
        auto end = chrono::steady_clock::now();
        cout << "Loop time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << endl;


        imshow("out", image);
        char key = (char) waitKey(waitTime);
        if(key == 27) break;

        tick = getTickCount();
        start = chrono::steady_clock::now();
    }

    return 0;
}
