#include "imu_array_simulation/gazebo_imu.h"
#include "imu_array_simulation/gazebo_odom.h"
#include "imu_array_simulation/kalman_filter.h"
#include "imu_array_simulation/ros_pub.h"

#include <csignal>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>

#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Float64MultiArray.h>

std::mutex cmd_vel_lock;
tf2::Vector3 cmd_vel;

void signalHandler(int signum) {
	ROS_WARN_STREAM("Interrupt signal received");

	exit(signum);
}

int main(int argc, char** argv) {
	signal(SIGINT, signalHandler);

	ros::init(argc, argv, "accel_kalman_filtering");

	double dt = 1./100.0;
	double ddt = dt*dt;
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::AsyncSpinner spinner(1);
	ros::Rate rate(1./dt);

	spinner.start();

	constexpr int nz = 3, nx = 9, nu = 1;

	GazeboIMU imu(nh, nh_priv, "center_imu");
	KalmanFilter kalman_filter(nz, nx, nu);

	PubROStopic<std_msgs::Float64MultiArray> estimated_pub(nh, "estimated_data");
	PubROStopic<std_msgs::Float64MultiArray> measured_pub(nh, "measured_data");

	ros::Duration(1.0).sleep();

	imu.SetAccelFilter(false); //gazebo_imu implemented KF internally.
	// imu.SetAccelXNoiseParam(0.0, 0.011, 0.0, 1.2e-6, 0.01);
	// imu.SetAccelYNoiseParam(0.0, 0.011, 0.0, 2.7e-6, 0.01);
	// imu.SetAccelZNoiseParam(0.0, 0.020, 0.0, 1.0e-6, 0.01);

	{
	Eigen::VectorXd X(nx);
	Eigen::MatrixXd P(nx, nx);
	Eigen::MatrixXd F(nx, nx);
	Eigen::MatrixXd B(nx, nu);
	Eigen::MatrixXd H(nz, nx);
	Eigen::MatrixXd Q(nx, nx);
	Eigen::MatrixXd R(nz, nz);

	X << 0., 0., 0., 0., 0., 0., 0., 0., 0.;

	P << 500., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 500., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 500., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 500., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 500., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 500., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 500., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 500., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 500.;

 	F << 1., dt, ddt/2., 0., 0., 0., 0., 0., 0.,
			 0., 1., dt, 0., 0., 0., 0., 0., 0.,
			 0., 0., 1., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 1., dt, ddt/2., 0., 0., 0.,
			 0., 0., 0., 0., 1., dt, 0., 0., 0.,
			 0., 0., 0., 0., 0., 1., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 1., dt, ddt/2.,
			 0., 0., 0., 0., 0., 0., 0., 1., dt,
			 0., 0., 0., 0., 0., 0., 0., 0., 1.;

	B << 0., 0., 0., 0., 0., 0., 0., 0., 0.;

	H << 0., 0., 1., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 1., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 1.;

	Q << 0., 0., 0., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 1., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 1., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0., 0., 0., 1.;
	Q = F*Q*F.transpose()*1.0e-6;

	R << 1.0e-5, 0., 0.,
	 		 0., 1.0e-5, 0.,
	 		 0., 0., 1.0e-5;

	kalman_filter.Init(X, P, F, B, H, Q, R);
	}

	while (ros::ok()) {
		Eigen::VectorXd z(nz);
		Eigen::VectorXd est_accel(nx);
		tf2::Vector3 accel(imu.GetAccel());
		std_msgs::Float64MultiArray estimated_data, measured_data;

		z << accel.getX(), accel.getY(), accel.getZ();
		kalman_filter.Update(z);

		est_accel = kalman_filter.GetEstState();

		kalman_filter.Predict( Eigen::MatrixXd::Identity(1,nu) );

		measured_data.data.push_back(accel.getX());
		measured_data.data.push_back(accel.getY());
		measured_data.data.push_back(accel.getZ());

		estimated_data.data.push_back(est_accel(2));
		estimated_data.data.push_back(est_accel(5));
		estimated_data.data.push_back(est_accel(8));

		std::cout << "    accel x: " << accel.getX() << " y: " << accel.getY() << " z: " << accel.getZ() << std::endl;
		std::cout << "est accel x: " << est_accel(2) << " y: " << est_accel(5) << " z: " << est_accel(8) << std::endl << std::endl;

		estimated_pub.ROS_Publish(estimated_data);
		measured_pub.ROS_Publish(measured_data);

		rate.sleep();
	}

	return 0;
}
