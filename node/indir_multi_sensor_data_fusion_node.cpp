#include "imu_array_simulation/gazebo_imu.h"
#include "imu_array_simulation/gazebo_odom.h"
#include "imu_array_simulation/kalman_filter.h"
#include "imu_array_simulation/ros_pub.h"

#include <csignal>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <array>

#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Float64.h>

void signalHandler(int signum) {
	ROS_WARN_STREAM("Interrupt signal received");

	exit(signum);
}

int main(int argc, char** argv) {
	signal(SIGINT, signalHandler);

	ros::init(argc, argv, "multi_sensor_est");

	double dt = 1./100.;
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::AsyncSpinner spinner(1);
	ros::Rate rate(1./dt);

	spinner.start();

	constexpr int nz = 3, nx = 17, nu = 1;
	constexpr double L = 0.3;
	double x, y, heading;

	Eigen::MatrixXd F1(nx, 4), F2(nx, 5), F3(nx, 8);
	Eigen::MatrixXd H1(nz, 5), H2(nz, 4), H3(nz, 8);
	std::shared_ptr<Eigen::VectorXd> X(new Eigen::VectorXd(nx));
	std::shared_ptr<Eigen::MatrixXd> P(new Eigen::MatrixXd(nx, nx));
	std::shared_ptr<Eigen::MatrixXd> F(new Eigen::MatrixXd(nx, nx));
	std::shared_ptr<Eigen::MatrixXd> B(new Eigen::MatrixXd(nx, nu));
	std::shared_ptr<Eigen::MatrixXd> H(new Eigen::MatrixXd(nz, nx));
	std::shared_ptr<Eigen::MatrixXd> Q(new Eigen::MatrixXd(nx, nx));
	std::shared_ptr<Eigen::MatrixXd> R(new Eigen::MatrixXd(nz, nz));

	ros::Time last_t;
	GazeboIMU imu(nh, nh_priv, "center_imu");
	GazeboOdom odom( nh, nh_priv, "/odom");
	KalmanFilter kalman_filter(nz, nx, nu, X, P, F, B, H, Q, R);

	PubROStf tf_pub(nh);

	ros::Duration(1.0).sleep();

	// imu.SetAccelFilter(false);
	imu.SetAccelXNoiseParam(0.0, 0.011, 0.0, 1.2e-6, 0.01);
	imu.SetAccelYNoiseParam(0.0, 0.011, 0.0, 2.7e-6, 0.01);
	imu.SetAccelZNoiseParam(0.0, 0.020, 0.0, 1.0e-6, 0.01);

	imu.SetGyroXNoiseParam(0.0, 0.75, 0.0, 4.0e-5, 0.01);
	imu.SetGyroYNoiseParam(0.0, 0.78, 0.0, 3.9e-5, 0.01);
	imu.SetGyroZNoiseParam(0.0, 0.79, 0.0, 7.3e-5, 0.01);

	// imu.SetAccelXNoiseParam(0.0, 0.0, 0.0, 0.0, 0.01);
	// imu.SetAccelYNoiseParam(0.0, 0.0, 0.0, 0.0, 0.01);
	// imu.SetAccelZNoiseParam(0.0, 0.0, 0.0, 0.0, 0.01);
	//
	// imu.SetGyroXNoiseParam(0.0, 0.0, 0.0, 0.0, 0.01);
	// imu.SetGyroYNoiseParam(0.0, 0.0, 0.0, 0.0, 0.01);
	// imu.SetGyroZNoiseParam(0.0, 0.0, 0.0, 0.0, 0.01);

	// odom.SetOdom( tf2::Vector3(org_x, org_y, org_z), tf2::Quaternion(0.0, 0.0, 0.0, 1.0) );
	odom.SetDiffL(L);
	odom.SetTransNoiseParam(0.0, 0.3);
	odom.SetRotNoiseParam(0.0, (M_PI/180.0));
	odom.SetbiasParam(0.02, 0.0, M_PI*0.01);

	*X << Eigen::MatrixXd::Constant(nx, 1, 0.0);
	(*X)(7) = 0.02+M_PI*0.01; (*X)(8) = 0.02-M_PI*0.01; // Sl, Sr
	(*X)(10) = 0.0; (*X)(11) = 0.01; (*X)(12) = 0.0; (*X)(13) = 0.01; //Sax, Bax, Say, Bay
	(*X)(14) = 0.0; (*X)(15) = 0.01; // Sgz, Bgz
	*P << 500.*Eigen::MatrixXd::Identity(nx, nx);
	*B << Eigen::MatrixXd::Constant(nx, 1, 0.);
	*Q << Eigen::MatrixXd::Identity(nx, nx);
	*R << Eigen::MatrixXd::Identity(nz, nz);
	F1 << Eigen::MatrixXd::Zero(nx, 4); F1(36) = 1.; F1(54) = 1.;
	F2 << Eigen::MatrixXd::Zero(nx, 5); F2(40) = 1.; F2(58) = 1.; F2(76) = 1.;
	F3 << Eigen::MatrixXd::Zero(9, 8), Eigen::MatrixXd::Identity(8, 8); F3(36) = 1.; F3(71) = 1.; F3(107) = 1.; F3(125) = 1.;
	H1 << Eigen::MatrixXd::Zero(nz, 5); H1(6) = 1.; H1(10) = 1.;
	H2 << Eigen::MatrixXd::Zero(nz, 4);
	H3 << Eigen::MatrixXd::Zero(nz, 8); H3(6) = 1.; H3(13) = 1.; H3(20) = 1.;
	*F << F1, F2, F3;
	*H << H1, H2, H3;

	x = odom.GetOffset().getOrigin().getX();
	y = odom.GetOffset().getOrigin().getY();
	heading = tf2::getYaw(odom.GetPose().getRotation());
	last_t = ros::Time::now();

	double vxe_we = 0.0, vxe_sl = 0.0, vxe_sr = 0.0, vye_we = 0.0, vye_sl = 0.0, vye_sr = 0.0, we_sd = 0.0, we_sl = 0.0, we_sr = 0.0;
	double vxa_sx = 0.0, vxa_sy = 0.0, gyro = 0.0;

	while (ros::ok()) {
		// state
		Eigen::VectorXd z(nz);
		Eigen::VectorXd est_x(nx);

		// imu
		tf2::Vector3 accel(imu.GetAccel());

		// odom
		geometry_msgs::Twist vel = odom.GetVel();
		std::array<double, 2> wheel_vel = odom.GetDiffVel(); // vl=0, vr=1
		double vl = wheel_vel[0], vr = wheel_vel[1];
		double omega = vel.angular.z;

		// time
		ros::Time now_t = ros::Time::now();
		double dt = (now_t - last_t).toSec();
		last_t = now_t;

		double theta = vel.angular.z*dt; //tf2::getYaw(odom.GetPose().getRotation());
		double sintheta = sin(theta);
		double costheta = cos(theta);

		F2(0) = vxe_we;
		F2(1) = vye_we;
		F2(51) = vxe_sl;
		F2(52) = vye_sl;
		F2(55) = we_sl;
		F2(68) = vxe_sr;
		F2(69) = vye_sr;
		F2(72) = we_sr;

		F3(4) = we_sd;
		F3(19) = vxa_sx;
		F3(54) = we_sr;
		F3(90) = gyro;

		*F << F1, F2, F3;

		// encoder velocity error model
		vxe_we = ((vl+vr)*dt*sintheta)/2.;
		vxe_sl = (vl*costheta)/2.;
		vxe_sr = (vr*costheta)/2.;
		vye_we = -((vl+vr)*dt*costheta)/2.;
		vye_sl = (vl*sintheta)/2.;
		vye_sr = (vr*sintheta)/2.;
		we_sd = (vr - vl)/L;
		we_sl = vl/L;
		we_sr = -vr/L;

		// accelerometer error model
		vxa_sx = (accel.getX()*costheta + accel.getY()*sintheta)*dt;
		vxa_sy = (accel.getX()*sintheta + accel.getY()*costheta)*dt;

		// gyro error model
		gyro = imu.GetGyro().getZ();

		H1(12) = -vxe_we;
		H1(13) = -vye_we;

		H2(6) = -vxe_sl;
		H2(7) = -vye_sl;
		H2(8) = -we_sl;
		H2(9) = -vxe_sr;
		H2(10) = -vye_sr;
		H2(11) = -we_sr;

		H3(2) = -we_sd;
		H3(3) = vxa_sx;
		H3(10) = vxa_sy;
		H3(17) = gyro;

		*H << H1, H2, H3;
		z << 0., 0., 0.; // (*X)(0) - (*X)(2), (*X)(1) - (*X)(3), (*X)(4) - (*X)(5);

		kalman_filter.Update(z);

		est_x = kalman_filter.GetEstState();
		odom.PubOdom();

		kalman_filter.Predict(Eigen::MatrixXd::Zero(1, 1));

		// double v_idl = (vl + vr)/2.;
		// double w_idl = (vl - vr)/2.;
		double v_est = (vl + vr + (*X)(7)*vl + (*X)(8)*vr)/2.;
		double w_est = (vl - vr + (*X)(7)*vl - (*X)(8)*vr)/(L + (*X)(9)*L);
		// heading += omega*dt;
		// x += v_idl*cos(heading)*dt;
		// y += v_idl*sin(heading)*dt;
		heading += w_est*dt;
		x += v_est*cos(heading)*dt;
		y += v_est*sin(heading)*dt;
		tf2::Quaternion tmp_heading;
		tmp_heading.setRPY(0.0, 0.0, heading);
		tf_pub.ROS_Publish("odom", "est_base_footprint", tf2::Vector3(x, y, 0.0), tmp_heading);

		// std::cout << "X: \n" << (*X)(0) << "\n" << (*X)(1) << "\n" << (*X)(2) << "\n" << (*X)(3) << "\n\n";
		// std::cout << (*X)(4) << "\n" << (*X)(5) << "\n" << (*X)(6) << "\n" << (*X)(7) << "\n" << (*X)(8) << "\n\n";
		// std::cout << (*X)(9) << "\n" << (*X)(10) << "\n" << (*X)(11) << "\n" << (*X)(12) << "\n";
		// std::cout << (*X)(13) << "\n" << (*X)(14) << "\n" << (*X)(15) << "\n" << (*X)(16) << "\n\n";
		// std::cout << "F1: \n" << F1 << std::endl;
		// std::cout << "F2: \n" << F2 << std::endl;
		// std::cout << "F3: \n" << F3 << std::endl;
		// std::cout << "H1: \n" << H1 << std::endl;
		// std::cout << "H2: \n" << H2 << std::endl;
		// std::cout << "H3: \n" << H3 << std::endl;
		// std::cout << "v_idl: " << v_idl << "\nw_idl: " << w_idl << std::endl;
		// std::cout << "acc_x: " << vxa_sx << "\nacc_y: " << vxa_sy << "\ngyro_z: " << gyro << std::endl;
		// std::cout << "x7: " << (*X)(7) << "\nx8: " << (*X)(8) << "\nx9: " << (*X)(9) << std::endl;
		// std::cout << "vr: " << vr << "\nvl: " << vl << "\nomega: " << omega << "\nheading: " << heading << std::endl << std::endl;

		rate.sleep();
	}

	return 0;
}
