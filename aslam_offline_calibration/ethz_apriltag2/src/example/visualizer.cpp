#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>


// April tags detector and various families that can be selected by command line option
#include "apriltags/TagDetector.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

#include <cv_bridge/cv_bridge.h>


// 0 - front forward
// 1 - D435
// 2 - D435f
// 3 - D455 Bottom
// 4 - D455 Top
// 5 - Sick
double m_fx[6] = {609.6598510742188, 610.6261596679688, 611.9644775390625, 645.5792236328125, 648.2322998046875, 367.055999};
double m_px[6] = {314.3277893066406, 321.7332763671875, 315.26226806640625,649.4420166015625, 639.9681396484375, 254.640999};
double m_fy[6] = {608.3814086914062, 610.872314453125,  611.9193115234375, 644.9020385742188, 647.4974975585938, 367.192999};
double m_py[6] = {236.36318969726562,230.41363525390625,248.76666259765625,361.5071105957031, 361.8551940917969, 209.070999};
double m_tagSize = 0.05;

float rvecs[6][3] = {{1.209075850468334,-1.209292792434484,1.209057901078036},
                    {1.458191389125308,0.8319974340444612,2.263643604048535},
                    {1.455100338982649,0.8405513946649108,2.265747207554073},
                    {1.577149962124753,0.7316680925215131,2.238632687593462},
                    {1.233578139624455, -1.222582006462467, 1.209128157377013},
                    {1.24012534268928,-1.253028070010616,1.219715437257459}};
float tvecs[6][3] = {{0,0,0},
                    {0.5934097079582054,0.09020131633416784,0.05443937162631655},
                    {0.5927660839941474,0.1160712227512125,0.04591749513120478},
                    {0.5785502775010415,-0.03559233072227274,0.0894448547793775},
                    {0.09126516460009908, 0.2050268558651533, -0.03755998339186287},
                    {-0.1539589640318955,0.2625507912876486,-0.01979409581822188}};
float distortions[6][5] = {{0,0,0,0,0},
                            {0,0,0,0,0},
                            {0,0,0,0,0},
                            {0,0,0,0,0},
                            {0,0,0,0,0},
                            {-0.071381,0.216571,0,0,0}};


// here is were everything begins
int main(int argc, char* argv[]) {

  std::vector<std::string> topics;
  std::vector<int> cameraIndexes;
  
  bool frontForward = false;
  bool d435 = false;
  bool d435f = false;
  bool d455bot = true;
  bool d455top = false;
  bool sick = false;

  if (frontForward) {
    topics.push_back(std::string("/front_forward/color/image_raw"));
    cameraIndexes.push_back(0);
  } else if (d435) {
    topics.push_back(std::string("/D435/color/image_raw"));
    cameraIndexes.push_back(1);
  } else if (d435f) {
    topics.push_back(std::string("/D435f/color/image_raw"));
    cameraIndexes.push_back(2);
  } else if (d455bot) {
    topics.push_back(std::string("/D455_bottom/color/image_raw"));
    cameraIndexes.push_back(3);
  } else if (d455top) {
    topics.push_back(std::string("/D455_top/color/image_raw"));
    cameraIndexes.push_back(4);
  } else if (sick) {
    topics.push_back(std::string("/sick_visionary_t_mini/intensity"));
    cameraIndexes.push_back(5);
  }



    float camInverseMatrixData[9] = {-0.0027190152883243, -0.02300755396479698, 0.999731593687235,
 -0.9999249695885257, -0.01187845488182604, -0.002992908786812298,
 0.01194412613997409, -0.9996647211791447, -0.02297352999813079};
    cv::Mat camInverseMatrix {3, 3, CV_32F, camInverseMatrixData};
    cout << camInverseMatrix << endl;
    float inverseMatrix[9] = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    //float inverseMatrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    cv::Mat cameraMatrixBase {3, 3, CV_32F, inverseMatrix};
    cv::Mat baseFrame = camInverseMatrix * cameraMatrixBase;
    cout << baseFrame << endl;
    return 0;


//   rosbag::Bag bag;
//   bag.open("/home/dimitri/bags/sfnAtlGimli/day2/localization/calibration_start.bag", rosbag::bagmode::Read);

  

//   rosbag::View view(bag, rosbag::TopicQuery(topics));
//   cv_bridge::CvImagePtr cv_ptr;

//   const char* window_name_first = "first";
//   cv::namedWindow(window_name_first, 1);

//   cv::Mat imageCopy;
//   cv::Mat imageCopy2;

//   foreach(rosbag::MessageInstance const m, view)
//   {
//     int topicIndex = 0;
//     for(int i = 0; i < topics.size(); i++){
//       if (m.getTopic() == topics[i]) {
//         topicIndex = i;
//       }
//     }
//     sensor_msgs::ImageConstPtr s = m.instantiate<sensor_msgs::Image>();
//     cv::Mat image_gray;
//     cv::Mat imageCopy;
//     try{
//       if(m.getTopic() == "/sick_visionary_t_mini/intensity"){
//         cv_ptr = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::TYPE_16UC1);
//         cv_ptr->image.convertTo(image_gray, CV_8UC1, 0.3);
//         cv::cvtColor(image_gray,imageCopy,cv::COLOR_GRAY2RGB);
//       } else {
//         cv_ptr = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::BGR8);
//         cv::cvtColor(cv_ptr->image, image_gray, cv::COLOR_BGR2GRAY);
//         imageCopy = cv_ptr->image;
//         imageCopy2 = imageCopy;
//         break;
//         //imshow(window_name_first,imageCopy);
//       }
//     }
//     catch (cv_bridge::Exception& e){
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       continue;
//     }
//     //int k = cv::waitKey(0);
//   }
//   bag.close();


//   std::vector<cv::Point3f> objPts;
//   std::vector<cv::Point2f> imgPts;
//   cv::Matx33f cameraMatrix(
//         m_fx[cameraIndexes[0]], 0, m_px[cameraIndexes[0]],
//         0, m_fy[cameraIndexes[0]], m_py[cameraIndexes[0]],
//         0,  0,  1);
//   cv::Vec4f distParam(0,0,0,0); // all 0?

//   std::vector<cv::Point3d> objectPoints;
//   objectPoints.push_back(cv::Point3d(6.5,-0.5,-0.88));
//   objectPoints.push_back(cv::Point3d(6.5,0.5,-0.88));
//   objectPoints.push_back(cv::Point3d(5,0.5,-0.88));
//   objectPoints.push_back(cv::Point3d(3.6,-0.35,-0.88));
//   cv::Matx31f rvec(rvecs[cameraIndexes[0]]);
//   cv::Matx31f tvec(tvecs[cameraIndexes[0]]);
//   cout << rvec << endl;
//   cout << tvec << endl;
//   std::vector<cv::Point2d> projectedPoints;
//   cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distParam, projectedPoints);
//   for(int z = 0; z < projectedPoints.size(); z++){
//     cv::circle(imageCopy2, cv::Point2f(projectedPoints[z].x, projectedPoints[z].y), 8, cv::Scalar(0,0,255,0), 2);
//     cout << projectedPoints[z].x << " " << projectedPoints[z].y << endl;
//   }
//   imshow(window_name_first, imageCopy2); // OpenCV call
//   int k = cv::waitKey(0);

  return 0;
}




