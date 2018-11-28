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
cv::Mat tcl = (Mat_<double>(3,1) << -0.0615468,-0.00366127,0.202977);
cv::Mat Rcl = (Mat_<double>(3,3) << -0.00271928,-0.980224,0.0159012,-0.00340085,0.00356939,-0.980469,0.980347,-0.0221692,0.0163553); 

cv::Mat K = (Mat_<double>(3, 3) << 601.1819494061277, 0, 329.1305015302001, 0.000000, 600.0406404279748, 238.3837396948256, 0.000000, 0.000000, 1.000000);
cv::Mat D = (Mat_<double>(4,1) << -0.0541737, -0.761986, 3.11624, -3.69709);

cv::Mat global_img_;
int begin_ray_id_;
string topic_detect_img_;
string topic_laser_seg_;
std_msgs::Header global_header_;

typedef Eigen::Matrix < double, 3, 1> Vector3f;
typedef Eigen::Matrix < double, 4, 1> Vector4f;
typedef message_filters::sync_policies::ApproximateTime<laser_segment::Insight_lidar_segments,sensor_msgs::Image> sync_policy_classification;

/* Rplidar A1 配置参数  */
double p_min_scanner_range_ = 0.15000000596;
double p_max_scanner_range_ = 6.0;
double p_scanner_angle_min_ = -3.12413907051;
double p_scanner_angle_max_ = 3.14159274101;
double p_scanner_angle_increment_ = 0.0174532923847;
int p_ranges_num_ = 360;


void reprojectTo_show(cv::Mat img,std::vector<Eigen::Vector3f> laser_points,std::vector<int> laser_idx);
void callback(const laser_segment::Insight_lidar_segmentsConstPtr &msg_segments,
			  const sensor_msgs::ImageConstPtr &msg_Img);
 




