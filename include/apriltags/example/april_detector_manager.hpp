/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

#pragma once
using namespace std;

#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>

const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [deviceID]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              disable graphics\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "apriltags/TagDetector.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"


// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
//#include "Serial.h"


#include <cmath>
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t);

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);
namespace atd{
  class AprilTagDetector {
    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;
    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;


  public:
    AprilTagDetector():
      m_tagDetector(NULL),
      m_tagCodes(AprilTags::tagCodes36h11),
      m_width(640),
      m_height(480),
      m_tagSize(0.166),
      m_fx(600.0),
      m_fy(600.0),
      m_px(float(m_width)/2.0),
      m_py(float(m_height)/2.0)
    {};


    void setup();
    void setImageSize(const int &_image_width, const int &_image_height);
    void setCameraParams(const double &_fx, const double &_fy, const double &_cx, const double &_cy);
    void setTagCodes(std::string s);
    void setTagSize(const double &_tagSize);
    void print_detection(AprilTags::TagDetection& detection) const ;
    std::vector<double> _extractTagInfo(AprilTags::TagDetection& detection) const;
    //std::vector<std::vector<double>> extractTagInfo(cv::Mat image);
    //cv::Mat getDetectedImage(cv::Mat image);
    void getDetectedInfo(const cv::Mat &image, cv::Mat &image_result, std::vector<std::vector<double>> &taginfo_vector, bool draw_flag);
  };
}
