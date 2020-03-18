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


#include "apriltags/example/april_detector_manager.hpp"
using namespace std;
using namespace atd;

// Needed for getopt / command line options processing
#include <unistd.h>

const char* window_name = "apriltags_demo";

inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
  }

void AprilTagDetector::setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
}

void AprilTagDetector::setImageSize(const int &_image_width, const int &_image_height) {
    m_width = _image_width;
    m_height = _image_height;
}

void AprilTagDetector::setCameraParams(const double &_fx, const double &_fy, const double &_cx, const double &_cy) {
    m_fx = _fx;
    m_fy = _fy;
    m_px = _cx;
    m_py = _cy;
}

void AprilTagDetector::setTagSize(const double &_tagSize) {
    m_tagSize = _tagSize;
}

void AprilTagDetector::setTagCodes(string s) {
  if (s=="16h5") {
    m_tagCodes = AprilTags::tagCodes16h5;
  } else if (s=="25h7") {
    m_tagCodes = AprilTags::tagCodes25h7;
  } else if (s=="25h9") {
    m_tagCodes = AprilTags::tagCodes25h9;
  } else if (s=="36h9") {
    m_tagCodes = AprilTags::tagCodes36h9;
  } else if (s=="36h11") {
    m_tagCodes = AprilTags::tagCodes36h11;
  } else {
    cout << "Invalid tag family specified" << endl;
    exit(1);
  }
}

void AprilTagDetector::print_detection(AprilTags::TagDetection& detection) const {
  cout << "  Id: " << detection.id
      << " (Hamming: " << detection.hammingDistance << ")";

  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                          translation, rotation);

  Eigen::Matrix3d F;
  F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
  Eigen::Matrix3d fixed_rot = F*rotation;
  double yaw, pitch, roll;
  wRo_to_euler(fixed_rot, yaw, pitch, roll);


  cout << "  distance=" << translation.norm()
      << "m, x=" << translation(0)
      << ", y=" << translation(1)
      << ", z=" << translation(2)
      << ", yaw=" << yaw
      << ", pitch=" << pitch
      << ", roll=" << roll
      << endl;

  // Also note that for SLAM/multi-view application it is better to
  // use reprojection error of corner points, because the noise in
  // this relative pose is very non-Gaussian; see iSAM source code
  // for suitable factors.
}

void AprilTagDetector::getDetectedInfo(const cv::Mat &image, cv::Mat &image_result, std::vector<std::vector<double>> &taginfo_vector, bool draw_flag) {
  cv::Mat image_gray;
  cv::Mat _image_result = image.clone();
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  std::vector<std::vector<double>> _taginfo_vector;
  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
  for (int i=0; i<detections.size(); i++) {
    if(draw_flag){
      detections[i].draw(_image_result);
      cv::waitKey(1);      
    } 
    _taginfo_vector.push_back(_extractTagInfo(detections[i]));
  }
  image_result = _image_result;
  taginfo_vector = _taginfo_vector;
}

std::vector<double> AprilTagDetector::_extractTagInfo(AprilTags::TagDetection& detection) const {
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                          translation, rotation);
  Eigen::Matrix3d F;
  F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
  Eigen::Matrix3d fixed_rot = F*rotation;
  double yaw, pitch, roll;
  wRo_to_euler(fixed_rot, yaw, pitch, roll);
  cv::Matx33d rotation_(
    fixed_rot(0,0), fixed_rot(0,1), fixed_rot(0,2), 
    fixed_rot(1,0), fixed_rot(1,1), fixed_rot(1,2), 
    fixed_rot(2,0), fixed_rot(2,1), fixed_rot(2,2)
  );
  cv::Mat rvec;  
  cv::Rodrigues(rotation_, rvec);
  std::vector<double> result_ary = {detection.id};  
  for(int i=0;i<4;i++){
    std::pair<float, float> p = detection.p[i];
    result_ary.push_back(p.first);
    result_ary.push_back(p.second);
  }
  result_ary.push_back(detection.cxy.first);
  result_ary.push_back(detection.cxy.second);

  return result_ary;
}

/*
std::vector<std::vector<double>> AprilTagDetector::extractTagInfo(cv::Mat image) {
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  std::vector<std::vector<double>> taginfo_vector;
  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
  for (int i=0; i<detections.size(); i++) {
    taginfo_vector.push_back(_extractTagInfo(detections[i]));

    if(detections[i].id == 5){
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      detections[i].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                              translation, rotation);

      Eigen::Matrix3d F;
      F <<
        1, 0,  0,
        0,  -1,  0,
        0,  0,  1;
      Eigen::Matrix3d fixed_rot = F*rotation;
      cv::Matx33d rotation_(
        fixed_rot(0,0), fixed_rot(0,1), fixed_rot(0,2), 
        fixed_rot(1,0), fixed_rot(1,1), fixed_rot(1,2), 
        fixed_rot(2,0), fixed_rot(2,1), fixed_rot(2,2)
      );

    }

  }
  return taginfo_vector;
}
*/