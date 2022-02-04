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

	ros::init(argc, argv, "gyro_kalman_filtering");

	double dt = 1./100.0;
	double ddt = dt*dt;
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::AsyncSpinner spinner(1);
	ros::Rate rate(1./dt);

	spinner.start();

	constexpr int nz = 3, nx = 6, nu = 1;

	GazeboIMU imu(nh, nh_priv, "center_imu");
	KalmanFilter kalman_filter(nz, nx, nu);

	PubROStopic<std_msgs::Float64MultiArray> estimated_pub(nh, "estimated_data");
	PubROStopic<std_msgs::Float64MultiArray> measured_pub(nh, "measured_data");

	ros::Duration(1.0).sleep();

	imu.SetGyroFilter(false); //gazebo_imu implemented KF internally.
	// imu.SetGyroXNoiseParam(0.0, 0.011, 0.0, 1.2e-6, 0.01);
	// imu.SetGyroYNoiseParam(0.0, 0.011, 0.0, 2.7e-6, 0.01);
	// imu.SetGyroZNoiseParam(0.0, 0.020, 0.0, 1.0e-6, 0.01);

	{
	Eigen::VectorXd x(nx);
	Eigen::MatrixXd P(nx, nx);
	Eigen::MatrixXd F(nx, nx);
	Eigen::MatrixXd B(nx, nu);
	Eigen::MatrixXd H(nz, nx);
	Eigen::MatrixXd Q(nx, nx);
	Eigen::MatrixXd R(nz, nz);

	x << 0., 0., 0., 0., 0., 0.;

	P << 500., 0., 0., 0., 0., 0.,
			 0., 500., 0., 0., 0., 0.,
			 0., 0., 500., 0., 0., 0.,
			 0., 0., 0., 500., 0., 0.,
			 0., 0., 0., 0., 500., 0.,
			 0., 0., 0., 0., 0., 500.;

 	F << 1., dt, 0., 0., 0., 0.,
			 0., 1., 0., 0., 0., 0.,
			 0., 0., 1., dt, 0., 0.,
			 0., 0., 0., 1., 0., 0.,
			 0., 0., 0., 0., 1., dt,
			 0., 0., 0., 0., 0., 1.;

	B << 0., 0., 0., 0., 0., 0.;

	H << 0., 1., 0., 0., 0., 0.,
			 0., 0., 0., 1., 0., 0.,
			 0., 0., 0., 0., 0., 1.;

	Q << 0., 0., 0., 0., 0., 0.,
 			 0., 1., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 1., 0., 0.,
 			 0., 0., 0., 0., 0., 0.,
 			 0., 0., 0., 0., 0., 1.;
	Q = F*Q*F.transpose()*1.0e-6;

	R << 1.0e-5, 0., 0.,
	 		 0., 1.0e-5, 0.,
	 		 0., 0., 1.0e-5;

	kalman_filter.Init(x, P, F, B, H, Q, R);
	}

	while (ros::ok()) {
		Eigen::VectorXd z(nz);
		Eigen::VectorXd est_gyro(nx);
		tf2::Vector3 gyro(imu.GetGyro());
		std_msgs::Float64MultiArray estimated_data, measured_data;

		z << gyro.getX(), gyro.getY(), gyro.getZ();
		kalman_filter.Update(z);

		est_gyro = kalman_filter.GetEstState();

		kalman_filter.Predict( Eigen::MatrixXd::Identity(1,nu) );

		measured_data.data.push_back(gyro.getX());
		measured_data.data.push_back(gyro.getY());
		measured_data.data.push_back(gyro.getZ());

		estimated_data.data.push_back(est_gyro(1));
		estimated_data.data.push_back(est_gyro(3));
		estimated_data.data.push_back(est_gyro(5));

		std::cout << "   theta x: " << est_gyro(0) << " y: " << est_gyro(2) << " z: " << est_gyro(4) << std::endl;
		std::cout << "    gyro x: " << gyro.getX() << " y: " << gyro.getY() << " z: " << gyro.getZ() << std::endl;
		std::cout << "est gyro x: " << est_gyro(1) << " y: " << est_gyro(3) << " z: " << est_gyro(5) << std::endl << std::endl;

		estimated_pub.ROS_Publish(estimated_data);
		measured_pub.ROS_Publish(measured_data);

		rate.sleep();
	}

	return 0;
}
