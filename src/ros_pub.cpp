#include <imu_array_simulation/ros_pub.h>

/*
 *
 * PubROStopic
 *
 */
template<typename T>
PubROStopic<T>::PubROStopic(ros::NodeHandle _nh, std::string _topic) : nh_(_nh), topic_name_(_topic) {
	pub_ = _nh.advertise<T>(_topic, 0);
}

template<typename T>
PubROStopic<T>::~PubROStopic() {}

template<typename T>
void PubROStopic<T>::ROS_Publish(T msg) {
	pub_.publish(msg);
}

/*
 *
 * PubROStf
 *
 */
PubROStf::PubROStf(ros::NodeHandle _nh) : nh_(_nh) {}

PubROStf::~PubROStf() {}

void PubROStf::ROS_Publish(std::string frame_id, std::string child_frame_id, tf2::Vector3 p, tf2::Quaternion q) {
	geometry_msgs::TransformStamped tf_trans;

	tf_trans.header.stamp = ros::Time::now();
	tf_trans.header.frame_id = frame_id;
	tf_trans.child_frame_id = child_frame_id;

	tf_trans.transform.translation.x = p.getX();
	tf_trans.transform.translation.y = p.getY();
	tf_trans.transform.translation.z = p.getZ();
	tf_trans.transform.rotation = tf2::toMsg(q);

	tf_broadcaster_.sendTransform(tf_trans);
}


template class PubROStopic<std_msgs::Float64MultiArray>;
