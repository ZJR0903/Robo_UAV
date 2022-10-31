
#include "unitree_nav_ros/libUnitreeNav.h"

using namespace std;

UnitreeNav::UnitreeNav()
{
	// 订阅目标点信息
	goal_sub_    = n.subscribe("/move_base_simple/goal", 1, &UnitreeNav::GoalCallBack, this);
	// 订阅GAZEBO中机器人信息
	// 订阅里程计
	odom_sub_    = n.subscribe("/airsim_node/drone_1/odom_local_ned", 1, &UnitreeNav::OdomCallBack, this);
	target_sub_  = n.subscribe("/target", 1, &UnitreeNav::TargetCallBack, this);
	// 发布未经处理的速度到spline_control节点
	cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("plan", 1);
	cmd_pub_ = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
	// 发布前向点
	chasing_pub_ = n.advertise<sensor_msgs::PointCloud>("chasing", 1);
	pose_pub_    = n.advertise<airsim_ros_pkgs::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 1);
	target_pub_  = n.advertise<geometry_msgs::Pose>("/MPC_target", 1);
	reference_circle_id_pub_  = n.advertise<std_msgs::Int8>("/id", 1);
	target_tf_pub_  = n.advertise<geometry_msgs::Point32>("/target_tf", 1);
	red_circle_pub_ = n.advertise<std_msgs::String>("/swap", 1);
	
	listener_    = new tf::TransformListener(n, ros::Duration(5.0), true);
	// 机器人初始状态是等待
	robot_index_ = WALKING;
	last_robot_index_ = WALKING;
	// 读取并打印config文件
	isReadConfig();
	// 打印Config文件
	printConfig();

	// add by zjr. 2022.0329
	// 链接到全局规划
	// add by zjr. 2022.04.14
	spline_planner_ = std::make_shared<SplinePlanner>();

	vector<vector<float>> reference_line(11, vector<float>(5));
	reference_line = 
		// { {0, 0, 0, 0, 0},
		// 	{14.87, -0.79, -2.74, -25, -25},
		// 	{37.68, -12.27, -1.02, -10, -10},
		// 	{67.42, -12.94, -0.3, 10, 10},
		// 	{94.44, -8.05, 0.02, -30, -30},
		// 	{113.95, -35.7, -0.34, -70, -70},
		// 	{121.77, -67.73, -3.83, -85, -85},
		// 	{121.80, -96, -7.39, -100, -100}
		// };

			{ {0, 0, 0, 0, 0},
				{14.87, -0.79, -2.74, -25, -25},
				{25, -5.7, -2.25, -17, 0}, //*
				{37.68, -12.27, -1.02, -10, -10},
				{67.42, -12.94, -0.3, 10, 10},
				{80, -9.2, -0.6, 20, 0}, //*
				{94.44, -8.05, 0.02, -30, -30},
				{108, -20.8, -0.56, -50, 0}, //*
				{113.95, -35.7, -0.34, -70, -70},
				{121.77, -67.73, -3.83, -85, -85},
				{121.80, -96, -7.39, -100, -100}
		};

	for (int i = 0; i < reference_line.size(); ++i)
	{
		AirSim_global_path_[i].x = reference_line[i][0];
		AirSim_global_path_[i].y = reference_line[i][1];
		AirSim_global_path_[i].z = reference_line[i][2];
		AirSim_global_path_[i].yaw = reference_line[i][3];
		AirSim_global_path_[i].E_yaw = reference_line[i][4];
		geometry_msgs::Point32 p;
		p.x = AirSim_global_path_[i].x;
		p.y = AirSim_global_path_[i].y;
		p.z = AirSim_global_path_[i].z;
		global_path_.points.push_back(p);
	}

	vector<vector<float>> reference_line_infer(11, vector<float>(5));
	reference_line_infer = 
			{ {0.5, 0, 0, 0, 0},
				{15.223, 1.103, -2.154, -25, -25},
				{0, 0, 0, 0, 0}, //*
				{38.68, -12.47, -0.95, -10, -10},
				{67.923, -12.85, -0.3, 10, 10},
				{0, 0, 0, 0, 0}, //*
				{94.02, -8.87, 0.02, -30, -30},
				{0, 0, 0, 0, 0}, //*
				{114.19, -36.66, -0.34, -70, -70},
				{121.77, -68.73, -3.83, -85, -85},
				{121.80, -96, -7.39, -100, -100}
	};

	for (int i = 0; i < reference_line_infer.size(); ++i)
	{
		geometry_msgs::Pose p;
		p.position.x = reference_line_infer[i][0];
		p.position.y = reference_line_infer[i][1];
		p.position.z = reference_line_infer[i][2];
		infer_global_path_.push_back(p);
	}
}

UnitreeNav::~UnitreeNav()
{
	ROS_INFO("UNITREE NAV NODE CLOSED");
	airsim_ros_pkgs::VelCmd objective_vel;
	objective_vel.twist.linear.x = 0;
	objective_vel.twist.linear.z = 0;
	objective_vel.twist.angular.z = 0;
	cmd_pub_.publish(objective_vel);
}

void UnitreeNav::Manager()
{
	ros::Rate loop_rate(20);
	
	while (ros::ok())
	{
		// global_planner_->initGlobalPlanner();
		// // updateGoalLocation();
		// //初始化全局规划

		// if (global_path_.points.size() > 0)
		// {
			execute();
		// }
		
		loop_rate.sleep();
		ros::spinOnce();
	}
}

bool UnitreeNav::isReadConfig()
{
	std::ifstream yaml_file;
	std::string config_name = "/home/zjr/catkin_ws/src/config/cfg/unitree_nav.txt";

	// 打开读取txt
	yaml_file.open(config_name);
	if (!yaml_file.is_open())
		return false;

	std::string str;
	while (std::getline(yaml_file, str))
	{
		std::stringstream ss(str);
		std::string yaml_info;
		ss >> yaml_info;
		if (yaml_info == "gazebo_robot_name") ss >> gazebo_robot_name_;
		else if (yaml_info == "first_turn_target_tolerance") ss >> first_turn_target_tolerance_;
		else if (yaml_info == "arrive_tolerance") ss >> arrive_tolerance_;
	}

	yaml_file.close();
	return true;
}

geometry_msgs::Pose UnitreeNav::getRealAirSimTarget(geometry_msgs::Pose AirSim_target, int closed_next_aim)
{
	boost::unique_lock<boost::recursive_mutex> lock(mutex_);
	geometry_msgs::Pose final_pose_map;

	cout << "===getRealAirSimTarget:===" << endl;
	if (closed_next_aim == 2 || closed_next_aim == 5 || closed_next_aim == 7) closed_next_aim += 1;
	static int last_closed_next_aim = closed_next_aim;
	geometry_msgs::Pose AirSim_target_base;
	initializeQuaternion(AirSim_target_base.orientation);
	cout << "closed_next_aim " << closed_next_aim << " last_closed_next_aim " << last_closed_next_aim << endl;
	geometry_msgs::Pose AirSim_target_map;
	transformPosePositioin("real_base_link", "map", AirSim_target, AirSim_target_map, listener_);
	initializeQuaternion(AirSim_target_map.orientation);

	cout << "odom " << odom_.pose.pose.position.x << " " << odom_.pose.pose.position.y << " " << odom_.pose.pose.position.z << endl;
	cout << "AirSim_target " << AirSim_target.position.x << " " << AirSim_target.position.y << " " << AirSim_target.position.z << endl;
	cout << "AirSim_target_map " << AirSim_target_map.position.x << " " << AirSim_target_map.position.y << " " << AirSim_target_map.position.z << endl;
	cout << "AirSim_global_path_ " << AirSim_global_path_[closed_next_aim].x << " " << AirSim_global_path_[closed_next_aim].y << " " << AirSim_global_path_[closed_next_aim].z << endl;
	double length = hypot3(AirSim_global_path_[closed_next_aim].x - AirSim_target_map.position.x, 
												 AirSim_global_path_[closed_next_aim].y - AirSim_target_map.position.y,
											 	 AirSim_global_path_[closed_next_aim].z - AirSim_target_map.position.z);
	cout << "length " << length << endl;
	double telo_length = (closed_next_aim != 6) ? 2.0 : 2.0;
	static vector<DestinationState> wait_for_check;
	if (wait_for_check.size() == 0)
	{
		cout << "====set :  " << endl;
		int next_aim = closed_next_aim;
		if (closed_next_aim == 2 || closed_next_aim == 5 || closed_next_aim == 7) next_aim = closed_next_aim + 1;
		cout << "next_aim " << next_aim << endl;
		final_pose_map = infer_global_path_[next_aim];
		geometry_msgs::Pose temp_p;
		initializeQuaternion(final_pose_map.orientation);
		transformPosePositioin("map", "real_base_link", final_pose_map, temp_p, listener_);
		cout << "infer_global_path_[1] " << infer_global_path_[1].position.x << " " << infer_global_path_[1].position.y << " " << infer_global_path_[1].position.z << endl;
		cout << "final_pose_map " << final_pose_map.position.x << " " << final_pose_map.position.y << " " << final_pose_map.position.z << endl;
		cout << "temp_p " << temp_p.position.x << " " << temp_p.position.y << " " << temp_p.position.z << endl;
	}

	if (AirSim_global_path_[closed_next_aim].E_yaw != 0 && length < telo_length)
	{
		DestinationState state;
		state.distance_robot_goal = length;
		state.pose = AirSim_target_map;
		wait_for_check.push_back(state);
	}

	gauss_num_ = wait_for_check.size();

	foo(wait_for_check);

	if (last_closed_next_aim != closed_next_aim)
		wait_for_check.clear();

	cout << "wait_for_check.size() " << wait_for_check.size() << endl;

	if (wait_for_check.size() >= 10)
	{
		for (int i = wait_for_check.size()-1; i > wait_for_check.size()-6; --i)
		{
			final_pose_map.position.x += wait_for_check[i].pose.position.x;
			final_pose_map.position.y += wait_for_check[i].pose.position.y;
			final_pose_map.position.z += wait_for_check[i].pose.position.z;
			cout << i << ">=10 " << final_pose_map.position.x << " " << final_pose_map.position.y << " " << final_pose_map.position.z << endl;
		}
		final_pose_map.position.x /= 5;
		final_pose_map.position.y /= 5;
		final_pose_map.position.z /= 5;
	}
	else if (wait_for_check.size() > 0)
	{
		for (int i = wait_for_check.size()-1; i >= 0; --i)
		{
			final_pose_map.position.x += wait_for_check[i].pose.position.x;
			final_pose_map.position.y += wait_for_check[i].pose.position.y;
			final_pose_map.position.z += wait_for_check[i].pose.position.z;
			cout << i << " <10" << final_pose_map.position.x << " " << final_pose_map.position.y << " " << final_pose_map.position.z << endl;
		}
		final_pose_map.position.x /= wait_for_check.size();
		final_pose_map.position.y /= wait_for_check.size();
		final_pose_map.position.z /= wait_for_check.size();
	}

	last_closed_next_aim = closed_next_aim;

	return final_pose_map;
}

void UnitreeNav::execute()
{
	geometry_msgs::Twist t;
	cmd_vel_pub_.publish(t);
	geometry_msgs::Pose MPC_target;

	if (!updateRobotLocation())
	{
		ROS_ERROR("CANNOT GET ROBOT LOCATION!");
	}

	int reference_target_global = findCurrentGoalRoute(global_path_, robot_current_state_, 1);

	int closed_aim;
	int closed_next_aim;
	double close_distance = 999;
	for (int i = 0; i < global_path_.points.size(); ++i)
	{
		if (hypot3(robot_current_state_.pose.position.x - global_path_.points[i].x,
							 robot_current_state_.pose.position.y - global_path_.points[i].y,
							 robot_current_state_.pose.position.z - global_path_.points[i].z) < close_distance)
		{
			close_distance = hypot3(robot_current_state_.pose.position.x - global_path_.points[i].x,
															robot_current_state_.pose.position.y - global_path_.points[i].y,
															robot_current_state_.pose.position.z - global_path_.points[i].z);
			closed_aim = i;
		}
	}

	if (reference_target_global > closed_aim)
	{
		closed_next_aim  = closed_aim + 1;
	}
	else closed_next_aim = closed_aim;

	if (robot_index_ == PASS) closed_next_aim = closed_next_aim_;

	static geometry_msgs::Pose final_pose_map = getRealAirSimTarget(AirSim_target_, closed_next_aim);
	if (robot_index_ != PASS)
		final_pose_map = final_pose_map_;
	geometry_msgs::Pose real_AirSim_target_;
	initializeQuaternion(final_pose_map.orientation);

	transformPosePositioin("map", "real_base_link", final_pose_map, real_AirSim_target_, listener_);
	cout << "closed_next_aim " << closed_next_aim << endl;
	cout << "final_pose_map " << final_pose_map.position.x << " " << final_pose_map.position.y << " " << final_pose_map.position.z << endl;
	cout << "real_AirSim_target_ " << real_AirSim_target_.position.x << " " << real_AirSim_target_.position.y << " " << real_AirSim_target_.position.z << endl;
	int reference_target = closed_next_aim;
	if (closed_next_aim == 10)
	{
		std_msgs::String str;
		str.data = " ";
		red_circle_pub_.publish(str);
	}
	
	geometry_msgs::Pose target_m;
	static geometry_msgs::Pose last_MPC_target = target_m;

	target_m.position.x = global_path_.points[reference_target].x;
	target_m.position.y = global_path_.points[reference_target].y;
	target_m.position.z = global_path_.points[reference_target].z;
	
	// 前向点转换到base_link系下
	geometry_msgs::Pose target_b_ref;
	initializeQuaternion(target_m.orientation);
	transformPosePositioin("map", "real_base_link", target_m, target_b_ref, listener_);

	double distance_robot_to_reference_circle = hypot3(target_b_ref.position.x, target_b_ref.position.y, target_b_ref.position.z);
	double length_closed_aim = hypot3(robot_current_state_.pose.position.x - global_path_.points[closed_aim].x,
						 robot_current_state_.pose.position.y - global_path_.points[closed_aim].y,
						 robot_current_state_.pose.position.z - global_path_.points[closed_aim].z);

	if (AirSim_global_path_[closed_aim].E_yaw != 0 &&
			hypot3(robot_current_state_.pose.position.x - global_path_.points[closed_aim].x,
						 robot_current_state_.pose.position.y - global_path_.points[closed_aim].y,
						 robot_current_state_.pose.position.z - global_path_.points[closed_aim].z) < 2 && closed_aim != 0) robot_index_ = PASS;
	else if (AirSim_global_path_[closed_aim].E_yaw != 0 && last_robot_index_ != PASS && 
						hypot3(robot_current_state_.pose.position.x - global_path_.points[closed_aim].x,
						robot_current_state_.pose.position.y - global_path_.points[closed_aim].y,
						robot_current_state_.pose.position.z - global_path_.points[closed_aim].z) < 4 && closed_aim != 0) robot_index_ = ADJUST_ORIENTATION;
	else robot_index_ = WALKING;

	last_robot_index_ = robot_index_;

	geometry_msgs::Pose target_real_base;
	if ((robot_index_ == ADJUST_ORIENTATION || robot_index_ == PASS)&&
			real_AirSim_target_.position.x > 0 &&
			closed_aim != 0 && 
			AirSim_global_path_[closed_next_aim].E_yaw != 0)
	{
		// target_real_base = AirSim_target_;
		target_real_base = real_AirSim_target_;

		if (target_real_base.position.x < 0)
			robot_index_ = WALKING;
	}
	else 
	{
		target_real_base = target_b_ref;
	}

	if (robot_index_ == PASS || robot_index_ == ADJUST_ORIENTATION)
	{
		if (target_real_base.position.x < 0)
			robot_index_ = WALKING;
	}

	// cout << "target_real_base " << target_real_base.position.x << " " << target_real_base.position.y << " " << target_real_base.position.z << endl;
	geometry_msgs::Point32 target_tf;
	target_tf.x = target_real_base.position.x;
	target_tf.y = target_real_base.position.y;
	if (robot_index_ == WALKING) cout << "WALKING" << endl;
	if (robot_index_ == ADJUST_ORIENTATION) cout << "ADJUST_ORIENTATION" << endl;
	if (robot_index_ == PASS) cout << "PASS" << endl;
	target_tf.z = (robot_index_ == PASS) ? 0 : target_real_base.position.z;
	target_tf_pub_.publish(target_tf);

	// add by zjr. 2022.04.14

	geometry_msgs::Pose target_base;
	transformPosePositioin("real_base_link", "base_link", target_real_base, target_base, listener_);
	double distance_robot_goal = hypot3(target_base.position.x, target_base.position.y, target_base.position.z);

	geometry_msgs::Point32 target;
	target.x = target_base.position.x;
	target.y = target_base.position.y;
	target.z = 1;
	sensor_msgs::PointCloud chasing;
	chasing.points.push_back(target);
	spline_target_ = spline_planner_->runSplineControl(target, distance_robot_goal);
	target_base.position.x = spline_target_.x;
	target_base.position.y = spline_target_.y;
	target_base.position.z = spline_target_.z;
	
	// 前向点转换到base_link系下
	initializeQuaternion(target_base.orientation);
	transformPosePositioin("base_link", "real_base_link", target_base, MPC_target, listener_);
	
	geometry_msgs::Point32 tt;
	tt.x = MPC_target.position.x;
	tt.y = MPC_target.position.y;
	tt.z = MPC_target.position.z;
	// double yy = atan2(tt.y, tt.x);
	double yy = atan2(target_real_base.position.y, target_real_base.position.x);
	chasing.points.push_back(tt);

	double odom_yaw = tf::getYaw(odom_.pose.pose.orientation);
	double e_yaw = (AirSim_global_path_[closed_next_aim].yaw / 180) * PI;
	double delta_yaw;
	if (odom_yaw > 0 && e_yaw > 0) delta_yaw = e_yaw - odom_yaw;
	else if (odom_yaw < 0 && e_yaw < 0) delta_yaw = e_yaw - odom_yaw;
	else if (odom_yaw < 0 && e_yaw > 0 && 
					 (fabs(odom_yaw) + e_yaw < (PI + odom_yaw + PI - e_yaw))) delta_yaw = fabs(odom_yaw) + e_yaw;
	else if (odom_yaw < 0 && e_yaw > 0 && 
					 (fabs(odom_yaw) + e_yaw > (PI + odom_yaw + PI - e_yaw))) delta_yaw = -(PI + odom_yaw + PI - e_yaw);
	else if (odom_yaw > 0 && e_yaw < 0 && 
					 (odom_yaw + fabs(e_yaw) < (PI + e_yaw + PI - odom_yaw))) delta_yaw = -(odom_yaw + fabs(e_yaw));
	else if (odom_yaw > 0 && e_yaw < 0 && 
					 (odom_yaw + fabs(e_yaw) > (PI + e_yaw + PI - odom_yaw))) delta_yaw = (PI + e_yaw + PI - odom_yaw);
	chasing.header.frame_id = "base_link";
	cout << "----------------------------" << endl;
	chasing_pub_.publish(chasing);
	// if (robot_index_ == ADJUST_ORIENTATION && closed_aim == closed_next_aim)
	// tt.z = AirSim_target_.position.z;
	tt.z = target_real_base.position.z;
	// if (robot_index_ == PASS)

	applySpeedFollow(tt, delta_yaw);
	closed_next_aim_ = closed_next_aim;
}

bool UnitreeNav::updateRobotLocation()
{
	tf::StampedTransform transform;

	try 
	{
		listener_->lookupTransform("map", "real_base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException &ex)
	{
		std::cerr << ex.what() << '\n';
		return false;
	}

	robot_current_state_.pose.position.x = transform.getOrigin().x();
	robot_current_state_.pose.position.y = transform.getOrigin().y();
	robot_current_state_.pose.position.z = transform.getOrigin().z();
	robot_current_state_.yaw = transform.getOrigin().z();
	// robot_current_state_.pose.position.z = tf::getYaw(transform.getRotation());
	return true;
}

bool UnitreeNav::updateGoalLocation()
{
	if (!transformPosePositioin("map", "base_link", robot_target_map_.pose, robot_target_base_.pose, listener_))
		return false;
	// 计算目标点与机器人的方位角
	robot_target_base_.tolerance_angle =
		atan2(robot_target_base_.pose.position.y, robot_target_base_.pose.position.x);
	// 计算目标点与机器人的距离
	robot_target_base_.distance_robot_goal =
		hypot(robot_target_base_.pose.position.x, robot_target_base_.pose.position.y);
	robot_target_map_.tolerance_angle = robot_target_map_.tolerance_angle;
	robot_target_map_.distance_robot_goal = robot_target_map_.distance_robot_goal;

	return true;
}

bool UnitreeNav::transformPosePositioin(std::string frame,
                                        std::string target_frame,
                                        geometry_msgs::Pose& input,
                                        geometry_msgs::Pose& output,
                                        tf::TransformListener* listener)
{
	geometry_msgs::PoseStamped input_s, output_s;
	input_s.header.frame_id = frame;
	input_s.pose = input;
	output_s.header.frame_id = target_frame;
	
	try
	{
		input_s.header.stamp = ros::Time(0);
		input_s.header.frame_id = frame;
		listener->transformPose(target_frame, input_s, output_s);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return false;
	}

	output = output_s.pose;

	return true;
}

bool UnitreeNav::isGoalReached()
{
	// 更新机器人位姿
	if (!updateRobotLocation())
	{
		ROS_ERROR("CANNOT GET ROBOT LOCATION!");
		return true;
	}
	// 机器人为等待状态时默认到达目标点
	if (robot_index_ == WAITTING)
		return true;

	// 更新目标点在base系下位姿
	if (!updateGoalLocation())
	{
		ROS_ERROR("CANNOT GET GOAL POSE IN BASE_LINK !");
		return false;
	}

	// 机器人处于首次转向状态
	if (robot_index_ == TURN_TARGET)
	{   
		// add zjr. 0823 
		if (robot_target_base_.distance_robot_goal <= 0.2)
		{
			// 机器人状态置为减速
			robot_index_ = SLOW_DOWN;
			
			// 是否减速完成
			if (isStop())
			{
				// 返回到达目标点
				return true;
				robot_index_ == WAITTING;
			}
		}
	}
	// 机器人处于行走状态
	else if (robot_index_ == WALKING) 
	{   
		// 判断机器人是否到达目标点
		if (robot_target_base_.distance_robot_goal <= 0.2)
		{
			// 机器人状态置为减速
			robot_index_ = SLOW_DOWN; 
			// 是否减速完成
			if (isStop()) 
			{
				// 返回到达目标点
				return true; 
				robot_index_ == WAITTING;
			}
		}
	}

	return false;
}

geometry_msgs::Point32 UnitreeNav::getTemporaryTargetInBaseLink()
{
	// 获取全局路径map系下前向点
	int num = findCurrentGoalRoute(global_path_, robot_current_state_, 1);
	
	geometry_msgs::Point32 target;
	geometry_msgs::Pose target_m;
	target_m.position.x = global_path_.points[num].x;
	target_m.position.y = global_path_.points[num].y;
	
	// 前向点转换到base_link系下
	geometry_msgs::Pose target_b;
	initializeQuaternion(target_m.orientation);
	transformPosePositioin("map", "base_link", target_m, target_b, listener_);

  target.x = target_b.position.x;
  target.y = target_b.position.y;

	return target;
}

// add by zjr. 2022.03.28
geometry_msgs::Twist UnitreeNav::applySpeedFollow(geometry_msgs::Point32 target, double yaw)
{
	cout << "===applySpeedFollow:===" << endl;
	geometry_msgs::Twist twist;
	airsim_ros_pkgs::VelCmd objective_vel;
	double length = hypot(target.x, target.y);
	double p_x, p_y, p_z, p_a;
	// p_x = 1.9;
	// p_y = 1.9;
	// p_z = 0.6;
	// p_a = 1.8;

	p_x = 1.6;
	p_y = 2.0;
	p_z = 1.0;
	p_a = 1.0;

	if (robot_index_ == PASS || robot_index_ == ADJUST_ORIENTATION)
	{
		p_x = 0.8;
		p_y = 1.6;		
		p_z = 1.6;
	}

	objective_vel.twist.linear.x = p_x * target.x;
	objective_vel.twist.linear.y = p_x * target.y;

	double time = length / fabs(objective_vel.twist.linear.x);
	
	objective_vel.twist.linear.z = p_z * target.z;
	objective_vel.twist.angular.z = p_a * yaw;

	// if (spline_planner_->isNeedBack()) 
	// {
	// 	objective_vel.twist.linear.x = -0.3;
	// 	objective_vel.twist.linear.y = target.y;
	// }

	cmd_pub_.publish(objective_vel);
	cout << "==========" << endl;
	cout << "target " << target.x << " " << target.y << " " << target.z << " " << yaw << endl;
	// cout << "length " << length << endl;
	cout << "velocity x " << objective_vel.twist.linear.x << endl;
	cout << "velocity y " << objective_vel.twist.linear.y << endl;
	cout << "velocity z " << objective_vel.twist.linear.z << endl;
	cout << "angular z " << objective_vel.twist.angular.z << endl;
	cout << "----------" << endl;

	// airsim_ros_pkgs::PoseCmd objective_pose;
	// objective_pose.roll = double(0.0);
	// objective_pose.pitch = double(atan(target.z / target.x));
	// objective_pose.yaw = double(atan(target.y / target.x));
	// objective_pose.throttle = 0.0;
	
	// pose_pub_.publish(objective_pose);

return twist;
}
// add by zjr. 2022.03.28
int UnitreeNav::findCurrentGoalRoute(const sensor_msgs::PointCloud& path, 
																	 	 const RobotState& robotState, 
																		 double lookahead) 
{
	geometry_msgs::Point32 Robot;
	Robot.x = robotState.pose.position.x;
	Robot.y = robotState.pose.position.y;
  geometry_msgs::Point32 current_goal;
  int output = -1;
  double min_value = DBL_MAX;
  double min_index_i = -1;
  double min_index_j = -1;
	// 全局路径为空返回
  if (path.points.size() <= 0)
    return 0;

	// 遍历全局路径
  for (int i = 0; i < path.points.size()-1; ++i) 
	{
		// 前向距离
    double lookahead_applied = lookahead;
		// 计算到每个连续三个点的距离
    double distance_i 
      = hypot((path.points.at(i).x - Robot.x),(path.points.at(i).y - Robot.y));
    double distance_j 
      = hypot((path.points.at(i+1).x - Robot.x),(path.points.at(i+1).y - Robot.y));
    double distance_ij 
      = hypot((path.points.at(i).x - path.points.at(i+1).x)
      ,(path.points.at(i).y - path.points.at(i+1).y));

		// 计算距离
    double value_ij = (distance_i + distance_j - distance_ij) / distance_ij;

		// 对比上一次计算结果
    if(value_ij < min_value) 
		{
      min_value = value_ij;
      min_index_i = i;
      min_index_j = i + 1;
			// 静态存储本次结果
      if(distance_j < lookahead_applied && min_index_j < path.points.size()-1) 
        min_index_j++;
    }
  }

	// 避免结果非法
  if(min_index_i == -1) min_index_i = path.points.size() - 1;
  if(min_index_j == -1) min_index_j = path.points.size() - 1;
  output = min_index_j;
	// 向后顺眼
	// (output == path.points.size()-1) ? output : output += 1;

  return output;
}

void UnitreeNav::printConfig()
{
	std::cout << "==================================" << std::endl;
	std::cout << "          Unitree Config          " << std::endl;
	std::cout << "==================================" << std::endl;
	std::cout << "gazebo_robot_name : " << gazebo_robot_name_ << std::endl;
	std::cout << "first_turn_target_tolerance : " << first_turn_target_tolerance_ << std::endl;
	std::cout << "arrive_tolerance : " << arrive_tolerance_ << std::endl;
	std::cout << "==================================" << std::endl;
}