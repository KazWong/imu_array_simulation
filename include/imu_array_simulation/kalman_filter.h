#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#define R_VectorXd Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor>
#define R_MatrixXd Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>

/*
 * @brief kalman filter by eigen
 *
 */
class KalmanFilter {
public:
	/*
	 * @brief Constructor
	 *
	 * @param _nz measurement (z) size
	 * @param _nx state (x) size
	 * @param _nu input (u) size
	 */
	KalmanFilter(int _nz, int _nx, int _nu);
	/*
	 * @brief Constructor
	 *
	 * @param _nz measurement (z) size
	 * @param _nx state (x) size
	 * @param _nu input (u) size
	 * @param _* kalman filter matrix
	 */
	KalmanFilter(int _nz, int _nx, int _nu,
		std::shared_ptr<Eigen::VectorXd> _X,
		std::shared_ptr<Eigen::MatrixXd> _P,
		std::shared_ptr<Eigen::MatrixXd> _F,
		std::shared_ptr<Eigen::MatrixXd> _B,
		std::shared_ptr<Eigen::MatrixXd> _H,
		std::shared_ptr<Eigen::MatrixXd> _Q,
		std::shared_ptr<Eigen::MatrixXd> _R);
	/*
	 * @brief Destructor
	 *
	 * @param
	 */
	~KalmanFilter();
	/*
	 * @brief initialize Kalman filter
	 *
	 * @param
	 */
	void Init(Eigen::VectorXd _X, Eigen::MatrixXd _P, Eigen::MatrixXd _F,
			Eigen::MatrixXd _B, Eigen::MatrixXd _H, Eigen::MatrixXd _Q, Eigen::MatrixXd _R);
	/*
	 * @brief Kalman filter predict
	 *
	 * @param
	 */
	void Predict(const Eigen::VectorXd& u);
	/*
	 * @brief Kalman filter update
	 *
	 * @param
	 */
	void Update(const Eigen::VectorXd& z);
	/*
	 * @brief get the estimated state
	 *
	 * @param
	 */
	Eigen::VectorXd GetEstState() const;

	double dt;
private:
	int nz_, nx_, nu_;
	// Time Update (Predict)
	std::shared_ptr<Eigen::MatrixXd> F_, B_, Q_, P_;
	std::shared_ptr<Eigen::VectorXd> X_;

	// Measurement Update (Correct)
	std::shared_ptr<Eigen::MatrixXd> H_, R_;
	Eigen::MatrixXd K_, I_;
};
