#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include <laser_segment/Insight_lidar_segments.h>
#include <laser_segment/Insight_lidar_segments.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace message_filters;

/* 雷达-相机外参，相机内参 */
cv::Mat tcl = (Mat_<double>(3,1) << 0.0259365,-0.0200982,-0.0914582);
cv::Mat Rcl = (Mat_<double>(3,3) << -0.0580782,-0.99819,-0.0155896,-0.0126269,
0.0163492,-0.999787,0.998232,-0.0578689,-0.0135535); 
cv::Mat K = (Mat_<double>(3, 3) << 1398.526435675626, 0, 974.6271798798929,0, \
 1399.471439435285, 533.3491663893035,0, 0, 1);
cv::Mat D = (Mat_<double>(4,1) << -0.101938, -0.0501564, 0.0323029, 0.0226299);
/*
//640x480
cv::Mat K_1 = (Mat_<double>(3, 3) <<601.1819494061277, 0, 329.1305015302001, 0.000000, \
 600.0406404279748, 238.3837396948256, 0.000000, 0.000000, 1.000000);
cv::Mat D_1 = (Mat_<double>(4,1) << -0.0541737, -0.761986, 3.11624, -3.69709);

//1280x720
cv::Mat K_2 = (Mat_<double>(3, 3) <<907.3179347878757, 0, 622.1428470715612, 0.000000, \
 910.7995408784911, 330.5347846342667, 0.000000, 0.000000, 1.000000);
cv::Mat D_2 = (Mat_<double>(4,1) << -0.120183, 0.0311701, -0.0129947, -0.00799496);

//1920x1080
cv::Mat K_3 = (Mat_<double>(3, 3) <<1398.526435675626, 0, 974.6271798798929,0, \
 1399.471439435285, 533.3491663893035,0, 0, 1);
cv::Mat D_3 = (Mat_<double>(4,1) << -0.101938, -0.0501564, 0.0323029, 0.0226299);
*/

cv::Mat global_img_;
int begin_ray_id_;
int min_seg_points_;
string topic_detect_img_ = "cropped_detection_Image";
string topic_laser_seg_ = "insight_laser_seg";
std_msgs::Header global_header_;

typedef Eigen::Matrix < double, 3, 1> Vector3f;
typedef Eigen::Matrix < double, 4, 1> Vector4f;
typedef message_filters::sync_policies::ApproximateTime<laser_segment::Insight_lidar_segments,sensor_msgs::Image> sync_policy_classification;

/* Rplidar A1 配置参数 
double p_min_scanner_range_ = 0.15000000596;
double p_max_scanner_range_ = 6.0;
double p_scanner_angle_min_ = -3.12413907051;
double p_scanner_angle_max_ = 3.14159274101;
double p_scanner_angle_increment_ = 0.0174532923847;
int p_ranges_num_ = 360;
 */
/*UST-10LX参数*/
double p_min_scanner_range_ = 0.019999999553;
double p_max_scanner_range_ = 30.0;
double p_scanner_angle_min_ =  -2.35619449615;
double p_scanner_angle_max_ = 2.35619449615;
double p_scanner_angle_increment_ = 0.00436332309619;
int p_ranges_num_ = 1081;


void reprojectTo_show(cv::Mat* img,std::vector<Eigen::Vector3f>* laser_points,std::vector<int>* laser_idx,const int& i);
void callback(const laser_segment::Insight_lidar_segmentsConstPtr& msg_segments,
			  const sensor_msgs::ImageConstPtr& msg_Img);
