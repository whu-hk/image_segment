#include <image_segment/image_segment.h>

int main(int argc,char **argv)
{
   ros::init(argc, argv, "laser_image");
   ros::NodeHandle nh_("");
   ros::NodeHandle nh_local_("~");
   nh_local_.param<int>("begin_Rayid",begin_ray_id_,150);
   ROS_INFO("start cropped_detection_Image & insight_laser_seg subscribe");
   message_filters::Subscriber<laser_segment::Insight_lidar_segments> segment_sub(nh_, topic_laser_seg_, 1);         
   message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_,topic_detect_img_, 1); 
   message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(10),segment_sub,image_sub);
   sync.registerCallback(boost::bind(&callback, _1, _2));   // 回调
   ros::spin();
   ros::shutdown();
   return 0;
}

void callback(const laser_segment::Insight_lidar_segmentsConstPtr &msg_segments,
			  const sensor_msgs::ImageConstPtr &msg_Img) 
{
	std::cout<<" "<<std::endl;
    ROS_INFO("Get pairs");
    global_header_ = msg_segments->header;
    std::cout<<"time gap is : "<<msg_segments->header.stamp.now() - msg_Img->header.stamp.now()<<std::endl;
    /*********************************************************************/
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_Img, "bgr8");
	cv::Mat img = cv_ptr->image;	
	std::vector<laser_segment::Insight_lidar_segment> segs;
	for(auto& seg : msg_segments->segments)
		if(seg.ray_id.front() >= begin_ray_id_)  
			segs.push_back(seg);
	std::cout<<"(callback) segments in_sight: "<<segs.size()<<std::endl;
	if(segs.size() == 0)
		ROS_INFO("No segment in sight");
	else
	{
		for(int i = 0;i < segs.size();i++)
		{ 
			std::vector<double> ranges_ = segs[i].ranges;
			std::vector<int> ray_id_ = segs[i].ray_id;
			std::vector<Eigen::Vector3f> laser_points;
			std::vector<int> laser_idx;
			for(int j = 0;j < ray_id_.size();j++)
			{
				double angle = p_scanner_angle_min_ + ray_id_[j] * p_scanner_angle_increment_;
				const auto& range = ranges_[j]; 
				if (range >= p_min_scanner_range_ && range <= p_max_scanner_range_) 
				{	   
					Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
					Eigen::Vector3f point;
					point << rotation * (range * Eigen::Vector3f::UnitX());
					laser_points.push_back(point);
					laser_idx.push_back(ray_id_[j]);
				}
			}
			if(laser_points.size() > 0)
			{
				reprojectTo_show(img,laser_points,laser_idx,i);
			}
			laser_points.clear();
			laser_idx.clear();
		}
	}
	cv::namedWindow("image_with_segments");
    cv::imshow("image_with_segments", img);
    cv::waitKey(1);
}

void reprojectTo_show(cv::Mat img,std::vector<Eigen::Vector3f> laser_points,std::vector<int> laser_idx,int seg_num)
{
    std::vector<Point2d> pts_uv;
  	std::vector<int> idx;
  	double count_coor_y;
  	if(laser_points.size() == laser_idx.size()  && laser_idx.size() > 0 && laser_idx.size() <= 360)
  	{
	  	for (int i = 0; i < laser_points.size(); ++i)
      	{
	 		Eigen::Vector3f point = laser_points[i];
		 	Mat laser_point(3,1, CV_64FC1);
	 	 	laser_point.at<double>(0,0) = point[0];
	 	 	laser_point.at<double>(1,0) = point[1];
	 	 	laser_point.at<double>(2,0) = point[2];
     	 	cv::Mat point_c = Rcl * laser_point + tcl;
     	 	if(point_c.at<double>(2,0) <= 0.)  //why ???
         	 	continue;
     	 	point_c.at<double>(0,0) /=  point_c.at<double>(2,0);
      		 point_c.at<double>(1,0) /=  point_c.at<double>(2,0);
      	 	point_c.at<double>(2,0) = 1.0;
      	 	cv::Mat uv = K * point_c;
		 	if(uv.at<double>(0,0) >= 25 && uv.at<double>(0,0) <= 615 && uv.at<double>(1,0) >= 0 && uv.at<double>(1,0) <= 480)
		 	{
			 	cout << "pixel coordinate:("<< uv.at<double>(0,0) << ","<< uv.at<double>(1,0)<<")"<<std::endl;	 
     	     	Point2d pt_uv(uv.at<double>(0,0), uv.at<double>(1,0));
      	     	pts_uv.push_back(pt_uv);
		     	idx.push_back(laser_idx[i]);
			 	count_coor_y += uv.at<double>(1,0);
		 	}
     	}
   	 	cout <<"GET POINTS———— "<< pts_uv.size() << endl;
   	 	if(pts_uv.size() == 0)
   	   		return;
   	 	else
   	 	{
       		for (int j = 0; j < pts_uv.size(); ++j) 
            {
		  		cv::Point paint_point; 
		  		if(j == 0)
		  		{
			  		paint_point.x = int(pts_uv[j].x - 50);
		      		paint_point.y = int(pts_uv[j].y + 30);
		  	  		string ss =  "left:"+std::to_string(idx[j]);
		      		cv::putText(img, ss, paint_point ,cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255, 0), 1, 8); 
			  		cv::circle(img, pts_uv[j], 3, CV_RGB(0,0,255),2,8);
		 		}
		  		else if(j == pts_uv.size() - 1)
		  		{
			  		paint_point.x=int(pts_uv[j].x);
		      		paint_point.y=int(pts_uv[j].y - 30);
			  		string ss =  "right:"+std::to_string(idx[j]);
		      		cv::putText(img, ss, paint_point ,cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255, 0), 1, 8); 
			  		cv::circle(img, pts_uv[j], 3, CV_RGB(0,0,255),2,8);
		  		}
		  		else
		  		{		
		      		// cv::circle(img, pts_uv[j], 3, CV_RGB(255,255,0),-1,8);  //图像；圆心坐标；圆半径；圆颜色；圆线条的粗细，负数为填充效果；线条类型
	          		// cv::putText(img, std::to_string(idx[j]), paint_point ,cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,0, 255), 1, 8);
		  		}
	   		}
			cv::Point pointk;
			pointk.x = int(pts_uv[pts_uv.size() / 2].x - 10);
		    pointk.y = int(pts_uv[pts_uv.size() / 2].y - 10);
			string sentance =  "segment:"+std::to_string(seg_num);
	   		cv::putText(img,sentance,pointk,cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,255), 1, 8); 
	   		cv::line(img,pts_uv.front(),pts_uv.back(),CV_RGB(255,0,0),0.3,8);
       		pts_uv.clear();
       		idx.clear();
     	}
    }
  	else
     	ROS_INFO("points size not correct");
}
