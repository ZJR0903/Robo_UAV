
#ifndef LIB_VISULIZE_H
#define LIB_VISULIZE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/PoseCmd.h"
#include "airsim_ros_pkgs/Takeoff.h"
#include "airsim_ros_pkgs/Land.h"
#include "airsim_ros_pkgs/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <geometry_msgs/Point.h>
#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cfloat>
#include <fstream>
#include <ctime>
#include <string>
#include <bits/stdc++.h>

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>
#include <math.h>
#include <assert.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"


using namespace std;

// 124
struct TreeNode {
  int val;
  TreeNode *left;
  TreeNode *right;
  TreeNode() : val(0), left(nullptr), right(nullptr) {}
  TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
  TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
};

class Visualize //速度控制类
{
    public:
    Visualize();
    ~Visualize();
    /** Functions **/
    /**
     * @brief 管理unitree_nav_node节点的运行
     * 程序运行
     */
    void Execute();

    private:
    /** Const **/
    const int ROS_RATE_HZ = 50; //ROS更新频率

    /** Node Handle **/
    ros::NodeHandle n; // 节点句柄

		ros::Subscriber camera_left_sub_, camera_right_sub_, camera_bottom_sub_;
		ros::Publisher camera_point_pub_;
		cv::Mat img_left_;
		cv::Mat img_right_;

    /** Parameters **/

    /** Variables **/
		boost::recursive_mutex mutex_;
		
		void camerabottomCallback(const sensor_msgs::Image::ConstPtr& Input)
		{
			cv::Mat img_bottom_ = cv_bridge::toCvShare(Input, sensor_msgs::image_encodings::TYPE_8UC3)->image;
		}

		void cameraLeftCallback(const sensor_msgs::Image::ConstPtr& Input)
		{
			boost::unique_lock<boost::recursive_mutex> lock(mutex_);

			img_left_ = cv_bridge::toCvShare(Input, sensor_msgs::image_encodings::TYPE_8UC3)->image;
		}
		void cameraRightCallback(const sensor_msgs::Image::ConstPtr& Input)
		{
			boost::unique_lock<boost::recursive_mutex> lock(mutex_);

			img_right_ = cv_bridge::toCvShare(Input, sensor_msgs::image_encodings::TYPE_8UC3)->image;
		}
		sensor_msgs::PointCloud camera_pointcloud_;

		void updateEnvironment(cv::Mat left, cv::Mat right)
		{
			boost::unique_lock<boost::recursive_mutex> lock(mutex_);
			
			camera_pointcloud_.points.clear();
			if (img_left_.rows == 0 || img_right_.rows == 0)
				return;
			
			double fx = 320, fy = 320, cx = 320, cy = 240;
    	double b = 0.095;
			cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

			cv::Mat disparity_sgbm, disparity;
			sgbm->compute(left, right, disparity_sgbm);
			disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

			// 定义点云使用的格式：这里用的是XYZRGB
			
			// 根据视差和相机模型计算每一个点的三维坐标, 并添加到PCL点云中
			for (int v = 0; v < left.rows; v+=5)
				for (int u = 0; u < left.cols; u+=5) {
					if (disparity.at<float>(v, u) <= 2.0 || disparity.at<float>(v, u) >= 96.0) 
							continue;

					double depth = fx * b / (disparity.at<float>(v, u));
					// if (depth > 3) continue;

					geometry_msgs::Point32 p32;
					p32.y = depth * (u - cx) / fx;
					p32.z = depth * (v - cy) / fy;
					p32.x = depth;
					camera_pointcloud_.points.push_back(p32);
				}
			
			camera_pointcloud_.header.frame_id = "camera";
			camera_point_pub_.publish(camera_pointcloud_);

		}
}; // Visualize

#endif // LIB_VISULIZE_H
