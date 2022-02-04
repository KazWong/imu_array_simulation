#pragma once

#include <iostream>
#include <string.h>
#include <mutex>
#include <random>
#include <memory>
#include <thread>
#include <chrono>
#include <array>
#include <math.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

/*
 * @brief gazebo imu
 *
 */
class GazeboOdom {
public:
	/*
	 * @brief Constructor
	 *
	 * @param
	 */
	GazeboOdom(ros::NodeHandle _nh, ros::NodeHandle _nh_priv, std::string _odom_topic);
	/*
	 * @brief Destructor
	 *
	 */
	~GazeboOdom();
	/*
	 * @brief Functions to set translation and rotation noise
	 *
	 */
	void SetTransNoiseParam(double _white_noise_mean, double _white_noise_stddev);
 	void SetRotNoiseParam(double _white_noise_mean, double _white_noise_stddev);
	/*
	 * @brief Functions to set translation and rotation bias
	 *
	 */
	void SetbiasParam(double _x_bias, double _y_bias, double _w_bias);
	/*
	 * @brief Functions to get pose
	 *
	 */
	tf2::Transform GetPose();
	/*
	 * @brief Functions to get velocity
	 *
	 */
	geometry_msgs::Twist GetVel() const;
	/*
	 * @brief Functions to get differential wheel velocity
	 *
	 */
	std::array<double, 2> GetDiffVel() const;
	/*
	 * @brief Functions to get velocity
	 *
	 */
	tf2::Transform GetOffset() const;
	/*
	 * @brief Set odom to current position
	 *
	 */
	void SetOdom(tf2::Vector3 _p, tf2::Quaternion _q);
	/*
	 * @brief Set the distance between the centers of the two wheels
	 *
	 */
	void SetDiffL(double _l);
	/*
	 * @brief public odom
	 *
	 */
	void PubOdom();
	/*
	 * @brief Functions to get x, y, z
	 *
	 */
	double operator[](int idx);

private:
	/*
	 * @brief callback function to init variables
	 *
	 * @param msg imu data
	 */
	void Init_Callback(const nav_msgs::OdometryConstPtr& msg);
	/*
	 * @brief callback function to get odom value
	 *
	 * @param msg odom data
	 */
	void Callback(const nav_msgs::OdometryConstPtr& msg);
	/*
	 * @brief generate translation noise
	 *
	 * @param msg odom data
	 */
	inline double trans_white_noise() { return ( *trans_white_noise_ )(generator_); }
	/*
	 * @brief generate rotation noise
	 *
	 * @param msg odom data
	 */
	inline double rot_white_noise() { return ( *rot_white_noise_ )(generator_); }

	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;

	tf2::Transform offset_;

	// true values receive from ROS odom topic
	tf2::Transform pose_;
	tf2::Transform last_pose_;
	geometry_msgs::Twist vel_;
	std::mutex mtx_;

	// Odom with noise
	tf2::Transform est_pose_;
	tf2::Transform last_est_pose_;
	ros::Time last_t_;

	// Odom noise model
	std::random_device rd_;
	std::default_random_engine generator_;
	std::unique_ptr<std::normal_distribution<double>> trans_white_noise_;
	std::unique_ptr<std::normal_distribution<double>> rot_white_noise_;
	double x_bias_, y_bias_, w_bias_;

	// Differential drive Vl, Vr out
	std::array<double, 2> diff_wheel_vel_;
	double diff_l_;

	std::string odom_topic_;
	ros::Subscriber odom_sub_;
	tf2_ros::TransformBroadcaster odom_broadcaster_;
	std::string odom_parent_;
};
