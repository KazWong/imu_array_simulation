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

	ros::init(argc, argv, "multi_array_imu");

	double dt = 1./100.0;
	double ddt = dt*dt;
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::AsyncSpinner spinner(1);
	ros::Rate rate(1./dt);

	spinner.start();

	constexpr int nz = 12, nx = 12, nu = 6;
	const int d1x = 0.2, d2x = -0.3, d3y = 0.4, d4y = 0.5;
	constexpr int size = 4;

	std::unique_ptr<GazeboIMU> imu[size];
	GazeboIMU ref_imu(nh, nh_priv, "center_imu");
	KalmanFilter kalman_filter(nz, nx, nu);

	PubROStopic<std_msgs::Float64MultiArray> estimated_accel(nh, "estimated_data");
	PubROStopic<std_msgs::Float64MultiArray> measured_accel(nh, "measured_data");

	std::random_device rd;
	std::default_random_engine gen(rd());
	std::normal_distribution<double> white_noise(0.0, 0.1);
	std::normal_distribution<double> bs_sigma(0.0, 1.0e-2);
	std::normal_distribution<double> bias(0.0, 0.1);

	for (int i=0;i<size;i++) {
		imu[i].reset(new GazeboIMU(nh, nh_priv, "imu_link1_"+std::to_string(i)));
		// imu[i]->SetAccelXNoiseParam(0.0, white_noise(gen), 0.0, bs_sigma(gen), bias(gen));
		// imu[i]->SetAccelYNoiseParam(0.0, white_noise(gen), 0.0, bs_sigma(gen), bias(gen));
		// imu[i]->SetAccelZNoiseParam(0.0, white_noise(gen), 0.0, bs_sigma(gen), bias(gen));

		// imu[i]->SetAccelXNoiseParam(0.0, 0.055, 0.0, 1.2e-3, 0.03);
		// imu[i]->SetAccelYNoiseParam(0.0, 0.055, 0.0, 2.7e-3, -0.03);
		// imu[i]->SetAccelZNoiseParam(0.0, 0.020, 0.0, 1.0e-3, 0.01);

		// imu[i]->SetAccelXNoiseParam(0.0, 0.011, 0.0, 1.2e-6, 0.01);
		// imu[i]->SetAccelYNoiseParam(0.0, 0.011, 0.0, 2.7e-6, 0.01);
		// imu[i]->SetAccelZNoiseParam(0.0, 0.020, 0.0, 1.0e-6, 0.01);
	}

	ros::Duration(1.0).sleep();

	{
	Eigen::VectorXd x(nx);
	Eigen::MatrixXd P(nx, nx);
	Eigen::MatrixXd A(nx, nx);
	Eigen::MatrixXd B(nx, nu);
	Eigen::MatrixXd H(nz, nx);
	Eigen::MatrixXd Q(nx, nx);
	Eigen::MatrixXd R(nz, nz);
	Eigen::VectorXd u(nu);

	x << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;

	u << 0., 0., 0., 0., 0., 0.;

	P << 500., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 500., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 500., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 500., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 500., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 500., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 500., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 500., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 500., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 500., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 500., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 500.;

 	A << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;

	H << 1., 0., 0., 0., 0., 0., 0., -d1x, -d1x, 0., 0., 0.,
			 1., 0., 0., 0., 0., 0., 0., -d2x, -d2x, 0., 0., 0.,
			 1., 0., 0., 0., 0., -d3y, 0., 0., 0., d3y, 0., 0.,
			 1., 0., 0., 0., 0., -d4y, 0., 0., 0., d4y, 0., 0.,
			 0., 1., 0., 0., 0., d1x, 0., 0., 0., d1x, 0., 0.,
			 0., 1., 0., 0., 0., d2x, 0., 0., 0., d2x, 0., 0.,
			 0., 1., 0., 0., 0., 0., -d3y, 0., -d3y, 0., 0., 0.,
			 0., 1., 0., 0., 0., 0., -d4y, 0., -d4y, 0., 0., 0.,
			 0., 0., 1., 0., -d1x, 0., 0., 0., 0., 0., 0., d1x,
			 0., 0., 1., 0., -d2x, 0., 0., 0., 0., 0., 0., d2x,
			 0., 0., 1., d3y, 0., 0., 0., 0., 0., 0., d3y, 0.,
			 0., 0., 1., d4y, 0., 0., 0., 0., 0., 0., d4y, 0.;

	B << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;

	Q << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;

	R << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
			 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;

	kalman_filter.Init(x, P, A, B, H, Q, R);
	}

	while (ros::ok()) {
		Eigen::VectorXd u(nu);
		Eigen::VectorXd z(nz);
		Eigen::VectorXd est_accel(nx);

		tf2::Vector3 ref_accel(ref_imu.GetAccel());
		std_msgs::Float64MultiArray estimated_data, measured_data;

		double accel_x[size], accel_y[size], accel_z[size];

		for (int i=0;i<size;i++) {
			tf2::Vector3 accel(imu[i]->GetAccel());
			accel_x[i] = accel.getX();
			accel_y[i] = accel.getY();
			accel_z[i] = accel.getZ();
			// std::cout << i << "     accel x: " << accel_x[i] << " y: " << accel_y[i] << " z: " << accel_z[i] << std::endl;
		}

		z << accel_x[0], accel_x[1], accel_x[2], accel_x[3],
				 accel_y[0], accel_y[1], accel_y[2], accel_y[3],
				 accel_z[0], accel_z[1], accel_z[2], accel_z[3];

		kalman_filter.Update(z);

		est_accel = kalman_filter.GetEstState();

		kalman_filter.Predict(u);

		// std::cout << "ref   accel x: " << ref_accel.getX() << " y: " << ref_accel.getY() << " z: " << ref_accel.getZ() << std::endl;
		// std::cout << "est   accel x: " << est_accel(0) << " y: " << est_accel(1) << " z: " << est_accel(2) << std::endl;
		// std::cout << "est   omega x: " << est_accel(3) << " y: " << est_accel(4) << " z: " << est_accel(5) << std::endl;
		// std::cout << "est  omega2 x: " << est_accel(6) << " y: " << est_accel(7) << " z: " << est_accel(8) << std::endl;
		// std::cout << "omega x omega x: " << est_accel(9) << " y: " << est_accel(10) << " z: " << est_accel(11) << std::endl << std::endl;

		measured_data.data.push_back(ref_accel.getX());
		measured_data.data.push_back(ref_accel.getY());
		measured_data.data.push_back(ref_accel.getZ());

		estimated_data.data.push_back(est_accel(0));
		estimated_data.data.push_back(est_accel(1));
		estimated_data.data.push_back(est_accel(2));

		estimated_accel.ROS_Publish(estimated_data);
		measured_accel.ROS_Publish(measured_data);

		rate.sleep();
	}

	return 0;
}
