#include <imu_array_simulation/gazebo_odom.h>

GazeboOdom::GazeboOdom(ros::NodeHandle _nh, ros::NodeHandle _nh_priv, std::string _odom_topic) :
 			nh_(_nh), nh_priv_(_nh_priv), odom_topic_(_odom_topic), rd_(), generator_(rd_()), diff_wheel_vel_{0.0, 0.0}, diff_l_(1.0) {
	ros::Subscriber odom_init_sub_ = nh_.subscribe<nav_msgs::Odometry>(_odom_topic, 0, &GazeboOdom::Init_Callback, this);

	trans_white_noise_.reset( new std::normal_distribution<double>(0.0, 0.0) );
	rot_white_noise_.reset( new std::normal_distribution<double>(0.0, 0.0) );

	tf2::Vector3 p(0.0, 0.0, 0.0);
	tf2::Quaternion q(0.0, 0.0, 0.0, 1.0) ;

	pose_.setOrigin(p); pose_.setRotation(q);
	last_pose_.setOrigin(p); last_pose_.setRotation(q);
	est_pose_.setOrigin(p); est_pose_.setRotation(q);
	last_est_pose_.setOrigin(p); last_est_pose_.setRotation(q);

	ros::Duration(1.0).sleep();

	last_pose_ = pose_ = est_pose_ = last_est_pose_ = offset_;

	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(_odom_topic, 0, &GazeboOdom::Callback, this);
}

GazeboOdom::~GazeboOdom() {}

void GazeboOdom::SetTransNoiseParam(double _white_noise_mean, double _white_noise_stddev) {
	trans_white_noise_.reset( new std::normal_distribution<double>(_white_noise_mean, _white_noise_stddev) );
}

void GazeboOdom::SetRotNoiseParam(double _white_noise_mean, double _white_noise_stddev) {
	rot_white_noise_.reset( new std::normal_distribution<double>(_white_noise_mean, _white_noise_stddev) );
}

void GazeboOdom::SetbiasParam(double _x_bias, double _y_bias, double _w_bias) {
	x_bias_ = _x_bias;
	y_bias_ = _y_bias;
	w_bias_ = _w_bias;
}

tf2::Transform GazeboOdom::GetPose() {
	mtx_.lock();
	tf2::Transform tf(est_pose_);
	mtx_.unlock();

	return tf;
}

geometry_msgs::Twist GazeboOdom::GetVel() const {
	return vel_;
}

std::array<double, 2> GazeboOdom::GetDiffVel() const {
	return diff_wheel_vel_;
}

tf2::Transform GazeboOdom::GetOffset() const {
	return offset_;
}

double GazeboOdom::operator[](int idx) {
	if (idx < 0 || idx >= 3) {
		throw std::range_error("Out of range");
	}

	switch(idx) {
		case 0: return est_pose_.getOrigin().getX();
		case 1: return est_pose_.getOrigin().getY();
		case 2: return est_pose_.getOrigin().getZ();
	}
}

void GazeboOdom::SetDiffL(double _l) {
	diff_l_ = _l;
}

void GazeboOdom::SetOdom(tf2::Vector3 _p, tf2::Quaternion _q) {
	mtx_.lock();
	est_pose_.setOrigin(_p); est_pose_.setRotation(_q);
	last_est_pose_ = est_pose_;
	last_pose_ = pose_;
	mtx_.unlock();
}

void GazeboOdom::PubOdom() {
	geometry_msgs::TransformStamped odom_trans;
	tf2::Quaternion q;
	tf2::Vector3 p;

	mtx_.lock();
	p = est_pose_.getOrigin();
	q = est_pose_.getRotation();
	mtx_.unlock();

	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = odom_parent_;
	odom_trans.child_frame_id = "noise_" + odom_parent_;

	odom_trans.transform.translation.x = p.getX();
	odom_trans.transform.translation.y = p.getY();
	odom_trans.transform.translation.z = p.getZ();
	odom_trans.transform.rotation = tf2::toMsg(q);

	odom_broadcaster_.sendTransform(odom_trans);
}

void GazeboOdom::Init_Callback(const nav_msgs::OdometryConstPtr& msg) {
	tf2::convert(msg->pose.pose, offset_);
}

void GazeboOdom::Callback(const nav_msgs::OdometryConstPtr& msg) {
	double x_noise = trans_white_noise();
	double y_noise = trans_white_noise();
	double w_noise = rot_white_noise();

	ros::Time now_t = ros::Time::now();
	double dt = (now_t - last_t_).toSec();
	last_t_ = now_t;

	tf2::Transform trans;
	tf2::Vector3 p(x_noise*dt*dt, y_noise*dt*dt, 0.0);
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, w_noise*dt*dt);
	tf2::Transform noise( q, p );

	double vx_bias = 0.0, vy_bias = 0.0, wz_bias = 0.0;

	if (dt > 0) {
		vx_bias = msg->twist.twist.linear.x*x_bias_;
		vy_bias = msg->twist.twist.linear.x*y_bias_;
		wz_bias = msg->twist.twist.linear.x*w_bias_;
	}

	double vel_x_noise = msg->twist.twist.linear.x + (x_noise + vx_bias)*dt;
	double vel_y_noise = msg->twist.twist.linear.y + (y_noise + vy_bias)*dt;
	double omega_noise = msg->twist.twist.angular.z + (w_noise + wz_bias)*dt;

	tf2::convert(msg->pose.pose, pose_);

	trans = last_pose_.inverse()*pose_;

	p = trans.getOrigin();
	q = trans.getRotation();
	p.setX(p.getX()*x_bias_);
	p.setY(p.getY()*y_bias_);
	q.setRPY(0.0, 0.0, tf2::getYaw(q)*w_bias_);
	tf2::Transform bias(q, p);

	mtx_.lock();
	est_pose_ = last_est_pose_*trans*noise*bias;

	vel_.linear.x = vel_x_noise;
	vel_.linear.y = vel_y_noise;
	vel_.angular.z = omega_noise;
	mtx_.unlock();

	// Differential wheel velocity outputZ
	double v = sqrt(vel_x_noise*vel_x_noise + vel_y_noise*vel_y_noise);
	v = (vel_x_noise >= 0.)? v:-v;

	diff_wheel_vel_[0] = (2*v + diff_l_*omega_noise)/2.; // vl
	diff_wheel_vel_[1] = (2*v - diff_l_*omega_noise)/2.; // vr

	last_pose_ = pose_;
	last_est_pose_ = est_pose_;
	odom_parent_ = msg->header.frame_id;
}
