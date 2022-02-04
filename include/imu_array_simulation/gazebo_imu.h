#pragma once

#include <iostream>
#include <string.h>
#include <mutex>
#include <random>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>

/*
 * @brief gazebo imu
 *
 */
class GazeboIMU {
public:
	/*
	 * @brief Constructor
	 *
	 * @param
	 */
	GazeboIMU(ros::NodeHandle nh, ros::NodeHandle nh_priv, std::string topic_name);
	/*
	 * @brief Destructor
	 *
	 */
	~GazeboIMU();
	/*
	 * @brief Functions to set each gyro axis noise
	 *
	 */
	void SetGyroXNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);
	void SetGyroYNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);
	void SetGyroZNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);
	/*
	 * @brief Functions to set each accel axis noise
	 *
	 */
	void SetAccelXNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);
	void SetAccelYNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);
	void SetAccelZNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);
	/*
	 * @brief Get Gyro data
	 *
	 */
	tf2::Vector3 GetGyro() const;
	/*
	 * @brief Get Accel data
	 *
	 */
	tf2::Vector3 GetAccel() const;
	/*
	 * @brief Set Gyro filter on/off
	 *
	 */
	void SetGyroFilter(bool on_off);
	/*
	 * @brief Set Accel filter on/off
	 *
	 */
	void SetAccelFilter(bool on_off);
	/*
	 * @brief Get by [], Gyro[0-2], Accel[3-5]
	 *
	 */
	double operator[](int idx);

private:
	/*
	 * @brief callback function to init variables
	 *
	 * @param msg imu data
	 */
	void Init_Callback(const sensor_msgs::ImuConstPtr& msg);
	/*
	 * @brief callback function to get imu value
	 *
	 * @param msg imu data
	 */
	void Callback(const sensor_msgs::ImuConstPtr& msg);
	/*
	 * @brief Kalman filter update function
	 *
	 * @param
	 */
	void KF_Predict(double dt);
	/*
	 * @brief Kalman filter predict function
	 *
	 * @param
	 */
	void KF_Update(Eigen::VectorXd& z);
	/*
	 * @brief function to set noise param
	 *
	 * @param _white_noise white noise sd
	 * @param _bs_sigma bs sd
	 * @param _bias_error bias error
	 */
	inline void SetNoiseParam(int idx, double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
				double _bias_error);

	tf2::Vector3 gyro_;
	tf2::Vector3 accel_;
	std::mutex mtx_;

	std::random_device rd_;
	std::default_random_engine generator_;

	//simulation offset
	double accel_offset_[3], gyro_offset_[3];

	// Kalman filter for simulation noise
	bool gyro_kf, accel_kf;
	int nz_, nx_, nu_;
	double last_t_;
	// Time Update (Predict)
	Eigen::MatrixXd F_, Q_, P_;
	Eigen::VectorXd X_;
	// Measurement Update (Correct)
	Eigen::MatrixXd H_, H_T_, K_, R_, I_;

	// Gyro and Accel noise model
	std::unique_ptr<std::normal_distribution<double>> white_noise_[6];
	std::unique_ptr<std::normal_distribution<double>> bs_sigma_[6];
	double bias_error_[6];
	double bias_instability_[6];


	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;
	ros::Subscriber imu_sub_;
};
