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


double front_forward_trans[3] = {0.548, 0.044, 0.883};
double front_forward_quat[4] = {0.007, -0.002, 0.001, 1.000};

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

double distortions[6][5] = {{0,0,0,0,0},
                            {0,0,0,0,0},
                            {0,0,0,0,0},
                            {-0.05492885038256645,0.06458774209022522,0,0.000738362199626863,-0.020988693460822105},
                            {-0.05617674067616463,0.06480415910482407,-0.00043705495772883296,0.0009292462491430342,-0.021189482882618904},
                            {0,0,0,0,0}};
                            //{-0.071381,0.216571,0,0,0}};

void wRo_to_euler(cv::Mat R, double& yaw, double& pitch, double& roll) {
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular){
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }else{
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    roll = x;
    pitch = y;
    yaw = z;
}


void calculateCameraCal(  std::vector<std::string> topics, std::vector<int> cameraIndexes){
  const char* window_name_first = "first";
  cv::namedWindow(window_name_first, 1);
  const char* window_name_seoncd = "second";
  cv::namedWindow(window_name_seoncd, 2);
  cv::Mat imageCopy;

  rosbag::Bag bag;
  bag.open("/home/dimitri/bags/sfnAtlGimli/day1/calibration.bag", rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  cv_bridge::CvImagePtr cv_ptr;
  vector<AprilTags::TagDetection> lastBaseDetections;
  ros::Time lastDetectionTime;
  cv::Mat lastBaseImage;
  vector<pair<AprilTags::TagDetection,AprilTags::TagDetection>> combinedDetections; 

  foreach(rosbag::MessageInstance const m, view)
  {

    
    int topicIndex = 0;
    for(int i = 0; i < topics.size(); i++){
      if (m.getTopic() == topics[i]) {
        topicIndex = i;
      }
    }
    sensor_msgs::ImageConstPtr s = m.instantiate<sensor_msgs::Image>();
    cv::Mat image_gray;
    try{
      if(m.getTopic() == "/sick_visionary_t_mini/intensity"){
        //cout << "lidar " << s->header.stamp << endl;
        cv_ptr = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.convertTo(image_gray, CV_8UC1, 0.3);
        cv::cvtColor(image_gray,imageCopy,cv::COLOR_GRAY2RGB);
      } else {
        //cout << "else " << s->header.stamp << endl;
        cv_ptr = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, image_gray, cv::COLOR_BGR2GRAY);
        imageCopy = cv_ptr->image;
      }
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      continue;
    }
    AprilTags::TagDetector* m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    for (int i=0; i<detections.size(); i++) {
      // also highlight in the image
      //detections[i].draw(imageCopy);
      //cout << detections[i].cxy.first << " " << detections[i].cxy.second << endl;
    }
    if(m.getTopic() == topics[0]){
      imshow(window_name_first, imageCopy); // OpenCV call
      lastBaseDetections = detections;
      lastDetectionTime = s-> header.stamp;
      if(topics.size() < 2){
        for(int i = 0; i < detections.size(); i++){
          pair<AprilTags::TagDetection, AprilTags::TagDetection> aPair = make_pair(detections[i], detections[i]);
          combinedDetections.push_back(aPair);
        }
      }
    } else {
      //imshow(window_name_seoncd, imageCopy); // OpenCV call
      for(int i = 0; i < detections.size(); i++){
        for (int j = 0; j <lastBaseDetections.size(); j++){
          if (detections[i].id == lastBaseDetections[j].id && abs(s->header.stamp.toSec() - lastDetectionTime.toSec()) < 0.05){
            //cout << "Match!: " << detections[i].id << endl;
            pair<AprilTags::TagDetection, AprilTags::TagDetection> aPair = make_pair(lastBaseDetections[j], detections[i]);
            combinedDetections.push_back(aPair);
            //cout << "matching pair: " << detections[i].id << endl;
            //cout << "first: " << lastBaseDetections[j].cxy.first << " " << lastBaseDetections[j].cxy.second << endl;
            //cout << "second: " << detections[i].cxy.first << " " << detections[i].cxy.second << endl;
          }
        }
      }
    }
  }
  bag.close();

  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;

  Eigen::Matrix4d transformMatrix = Eigen::Matrix<double, 4, 4>::Identity();
  transformMatrix(0,3) = front_forward_trans[0];
  transformMatrix(1,3) = front_forward_trans[1];
  transformMatrix(2,3) = front_forward_trans[2];
  // cout << "Front forward matrix:" << endl;
  // cout << transformMatrix << endl;
  int camerIndexToUse;
  if(topics.size() < 2){
    camerIndexToUse = 0;
  } else {
    camerIndexToUse = 1;
  }

  cv::Mat distParamsBase {5, 1, CV_64F, distortions[cameraIndexes[0]]};
  cv::Mat distParamsSecond {5, 1, CV_64F, distortions[cameraIndexes[camerIndexToUse]]};
  cout << distParamsBase << endl;
  for(int i = 0; i < combinedDetections.size(); i++){
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      combinedDetections[i].first.getRelativeTranslationRotation(m_tagSize, m_fx[cameraIndexes[0]], m_fy[cameraIndexes[0]], m_px[cameraIndexes[0]], m_py[cameraIndexes[0]],
                                              distParamsBase, translation, rotation);
      
      Eigen::Matrix4d T; 
      T.topLeftCorner(3,3) = rotation;
      T.col(3).head(3) << translation(0), translation(1), translation(2);
      T.row(3) << 0,0,0,1;
      //cout << T << endl;
      Eigen::Matrix4d T_transformed_with_front_forward = transformMatrix * T;
      objPts.push_back(cv::Point3f(T_transformed_with_front_forward(0,3),T_transformed_with_front_forward(1,3), T_transformed_with_front_forward(2,3)));
      imgPts.push_back(cv::Point2f(combinedDetections[i].second.cxy.first, combinedDetections[i].second.cxy.second));
  }
  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
        m_fx[cameraIndexes[camerIndexToUse]], 0, m_px[cameraIndexes[camerIndexToUse]],
        0, m_fy[cameraIndexes[camerIndexToUse]], m_py[cameraIndexes[camerIndexToUse]],
        0,  0,  1);
  cout << distParamsSecond << endl;
  cout << cameraIndexes[camerIndexToUse] << endl;
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParamsSecond, rvec, tvec);
  cv::Mat r;
  cv::Rodrigues(rvec, r);
  double yaw, pitch, roll;
  cout << "===========================================" << endl;
  cout << "Calibrating from " << topics[0] << " to: " << topics[topics.size()-1] << endl;
  cv::Mat cameraRotationInversed = r.t();
  cv::Mat cameraPosition = -cameraRotationInversed * tvec;
  cout << cameraPosition << endl;
  cout << tvec << endl;

  double inverseMatrix[9] = {0, 0, 1, -1, 0, 0, 0, -1, 0};
  cv::Mat cameraMatrixBase {3, 3, CV_64F, inverseMatrix};
  cv::Mat baseFrame = cameraRotationInversed * cameraMatrixBase.t();
  cout << "****************************************" << endl;
  cout << "Camera Base Frame:" << endl;
  cout << "Camera position: " << cameraPosition.at<double>(0,0) << " " << cameraPosition.at<double>(1,0) << " " << cameraPosition.at<double>(2,0) << endl;
  wRo_to_euler(baseFrame, yaw, pitch, roll);
  cout << "Camera angle: " << roll << " " << pitch << " " << yaw << endl;
  cout << endl;
  cout << "Camera Optical Frame:" << endl;
  wRo_to_euler(cameraMatrixBase, yaw, pitch, roll);
  cout << "Optical angle: " << roll << " " << pitch << " " << yaw << endl;
  cout << "****************************************" << endl;
  cout << "===========================================" << endl;
  cout << endl;

}

// here is were everything begins
int main(int argc, char* argv[]) {

  std::vector<std::string> topics;
  topics.push_back("/front_forward/color/image_raw");
  std::vector<int> cameraIndexes;
  cameraIndexes.push_back(0);
  //calculateCameraCal(topics, cameraIndexes);


  cout << "starting" << endl;
  topics.push_back(std::string("/D435/color/image_raw"));
  cameraIndexes.push_back(1);
  calculateCameraCal(topics, cameraIndexes);
  topics.erase(topics.begin()+1);
  cameraIndexes.erase(cameraIndexes.begin()+1);

  // topics.push_back(std::string("/D435f/color/image_raw"));
  // cameraIndexes.push_back(2);
  // calculateCameraCal(topics, cameraIndexes);
  // topics.erase(topics.begin()+1);
  // cameraIndexes.erase(cameraIndexes.begin()+1);

  topics.push_back(std::string("/D455_bottom/color/image_raw"));
  cameraIndexes.push_back(3);
  calculateCameraCal(topics, cameraIndexes);
  topics.erase(topics.begin()+1);
  cameraIndexes.erase(cameraIndexes.begin()+1);

  topics.push_back(std::string("/D455_top/color/image_raw"));
  cameraIndexes.push_back(4);
  calculateCameraCal(topics, cameraIndexes);
  topics.erase(topics.begin()+1);
  cameraIndexes.erase(cameraIndexes.begin()+1);

  topics.push_back(std::string("/sick_visionary_t_mini/intensity"));
  cameraIndexes.push_back(5);
  calculateCameraCal(topics, cameraIndexes);
  topics.erase(topics.begin()+1);
  cameraIndexes.erase(cameraIndexes.begin()+1);

  return 0;
}






