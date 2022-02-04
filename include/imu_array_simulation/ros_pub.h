#pragma once

#include <iostream>
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>

template<typename T>
class PubROStopic {
public:
	/*
	 * @brief Constructor
	 *
	 * @param
	 */
	PubROStopic(ros::NodeHandle _nh, std::string _topic);
	/*
	 * @brief Destructor
	 *
	 */
	~PubROStopic();
	/*
	 * @brief Functions public message
	 *
	 */
	void ROS_Publish(T msg);
private:
	ros::NodeHandle nh_;
	std::string topic_name_;
	ros::Publisher pub_;
};

class PubROStf {
public:
	/*
	 * @brief Constructor
	 *
	 * @param
	 */
	PubROStf(ros::NodeHandle _nh);
	/*
	 * @brief Destructor
	 *
	 */
	~PubROStf();
	/*
	 * @brief Functions public tf
	 *
	 */
	void ROS_Publish(std::string frame_id, std::string child_frame_id, tf2::Vector3 p, tf2::Quaternion q);
private:
	ros::NodeHandle nh_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;
};
