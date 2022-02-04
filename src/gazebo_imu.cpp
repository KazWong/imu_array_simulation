#include <imu_array_simulation/gazebo_imu.h>

GazeboIMU::GazeboIMU(ros::NodeHandle _nh, ros::NodeHandle _nh_priv, std::string topic_name) :
 			nh_(_nh), nh_priv_(_nh_priv), rd_(), generator_(rd_()),
			accel_offset_{0.0, 0.0, 0.0}, gyro_offset_{0.0, 0.0, 0.0},
			nz_(6), nx_(15), last_t_(0.0), gyro_kf(true), accel_kf(true),
			X_(nx_),
			F_(nx_, nx_),
			P_(nx_, nx_),
			Q_(nx_, nx_),
			R_(nz_, nz_),
			H_(nz_, nx_),
		 	K_(nx_, nz_),
			I_(Eigen::MatrixXd::Identity(nx_, nx_)) {
	constexpr double kp = 500.0, kr = 1.0e-5, kqa = 1.0e-6, kqg = 1.0e-6;

	ros::Subscriber imu_init_sub_ = nh_.subscribe<sensor_msgs::Imu>(topic_name, 0, &GazeboIMU::Init_Callback, this);

	for (int i=0;i<6;i++) {
		white_noise_[i].reset( new std::normal_distribution<double>(0.0, 0.0) );
		bs_sigma_[i].reset( new std::normal_distribution<double>(0.0, 0.0) );
		bias_error_[i] = 0.0;
		bias_instability_[i] = 0.0;
	}

	X_ << Eigen::MatrixXd::Zero(nx_,1);
	P_ << kp*Eigen::MatrixXd::Identity(nx_,nx_);
	R_ << kr*Eigen::MatrixXd::Identity(nz_,nz_);

	H_ << 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;
	H_T_ = H_.transpose();

 	Q_ << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., kqa, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., kqa, 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
			 	0., 0., 0., 0., 0., 0., 0., 0., kqa, 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., kqg, 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., kqg, 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., kqg;

	ros::Duration(1.0).sleep();

	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(topic_name, 0, &GazeboIMU::Callback, this);
}

GazeboIMU::~GazeboIMU() {}

void GazeboIMU::KF_Predict(double dt) {
	Eigen::MatrixXd F_T(nx_, nx_);
	double a = dt*dt/2.0;

	F_ << 1., dt, a, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 1., dt, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 1., dt, a, 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 1., dt, 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 1., dt, a, 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 1., dt, 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 1., dt, 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., dt, 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., dt,
				0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;
	F_T = F_.transpose();

	X_ = F_*X_;
	P_ = F_*P_*F_T + F_*Q_*F_T;
}

void GazeboIMU::KF_Update(Eigen::VectorXd& z) {
	// gain update
	Eigen::MatrixXd PpH_T = P_*H_T_;
	K_ = PpH_T*( (H_*PpH_T + R_).inverse() );

	// measurement and state update
	Eigen::MatrixXd Y = z - H_*X_;
	X_ = X_ + K_*Y;

	// covariance update
	Eigen::MatrixXd I_KH = (I_ - K_*H_);
	P_ = I_KH*P_*I_KH.transpose() + K_*R_*K_.transpose();
}

inline void GazeboIMU::SetNoiseParam(int idx, double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	white_noise_[idx].reset( new std::normal_distribution<double>(_white_noise_mean, _white_noise_stddev) );
	bs_sigma_[idx].reset( new std::normal_distribution<double>(_bs_sigma_mean, _bs_sigma_stddev) );
	bias_error_[idx] = _bias_error;
	bias_instability_[idx] = 0.0;
}

void GazeboIMU::SetGyroXNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	SetNoiseParam(0, _white_noise_mean, _white_noise_stddev, _bs_sigma_mean, _bs_sigma_stddev, _bias_error);
}

void GazeboIMU::SetGyroYNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	SetNoiseParam(1, _white_noise_mean, _white_noise_stddev, _bs_sigma_mean, _bs_sigma_stddev, _bias_error);
}

void GazeboIMU::SetGyroZNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	SetNoiseParam(2, _white_noise_mean, _white_noise_stddev, _bs_sigma_mean, _bs_sigma_stddev, _bias_error);
}

void GazeboIMU::SetAccelXNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	SetNoiseParam(3, _white_noise_mean, _white_noise_stddev, _bs_sigma_mean, _bs_sigma_stddev, _bias_error);
}

void GazeboIMU::SetAccelYNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	SetNoiseParam(4, _white_noise_mean, _white_noise_stddev, _bs_sigma_mean, _bs_sigma_stddev, _bias_error);
}

void GazeboIMU::SetAccelZNoiseParam(double _white_noise_mean, double _white_noise_stddev, double _bs_sigma_mean, double _bs_sigma_stddev,
			double _bias_error) {
	SetNoiseParam(5, _white_noise_mean, _white_noise_stddev, _bs_sigma_mean, _bs_sigma_stddev, _bias_error);
}

tf2::Vector3 GazeboIMU::GetGyro() const {
	if (isnan(gyro_.getX()) || isnan(gyro_.getX()) || isnan(gyro_.getX()))
		tf2::Vector3(0.0, 0.0, 0.0);
	else
		return gyro_;
}

tf2::Vector3 GazeboIMU::GetAccel() const {
	if (isnan(accel_.getX()) || isnan(accel_.getX()) || isnan(accel_.getX()))
		tf2::Vector3(0.0, 0.0, 0.0);
	else
		return accel_;
}

void GazeboIMU::SetGyroFilter(bool on_off) {
	gyro_kf = on_off;
}

void GazeboIMU::SetAccelFilter(bool on_off) {
	accel_kf = on_off;
}

double GazeboIMU::operator[](int idx) {
	if (idx < 0 || idx >= 6) {
		throw std::range_error("Out of range");
	}

	switch(idx) {
		case 0: return (isnan(gyro_.getX()))? 0.0:gyro_.getX();
		case 1: return (isnan(gyro_.getY()))? 0.0:gyro_.getY();
		case 2: return (isnan(gyro_.getZ()))? 0.0:gyro_.getZ();
		case 3: return (isnan(accel_.getX()))? 0.0:accel_.getX();
		case 4: return (isnan(accel_.getY()))? 0.0:accel_.getY();
		case 5: return (isnan(accel_.getZ()))? 0.0:accel_.getZ();
	}
}

void GazeboIMU::Init_Callback(const sensor_msgs::ImuConstPtr& msg) {
	accel_offset_[0] = msg->linear_acceleration.x;
	accel_offset_[1] = msg->linear_acceleration.y;
	accel_offset_[2] = msg->linear_acceleration.z - 9.80665;

	gyro_offset_[0] = msg->angular_velocity.x;
	gyro_offset_[1] = msg->angular_velocity.y;
	gyro_offset_[2] = msg->angular_velocity.z;
}

void GazeboIMU::Callback(const sensor_msgs::ImuConstPtr& msg) {
	double noise[6];
	Eigen::VectorXd z(nz_);
	tf2::Vector3 gyro_tmp, accel_tmp;
	double t = msg->header.stamp.toSec();
	double dt = t - last_t_;
	last_t_ = t;

	for (int i=0;i<6;i++) {
		bias_instability_[i] += ( *(bs_sigma_[i]) )(generator_)*dt;
		noise[i] = (bias_error_[i] + ( *(white_noise_[i]) )(generator_))*dt + bias_instability_[i];
	}

	tf2::convert(msg->linear_acceleration, accel_tmp);
	tf2::convert(msg->angular_velocity, gyro_tmp);

	accel_tmp.setX(accel_tmp.getX() - accel_offset_[0]);
	accel_tmp.setY(accel_tmp.getY() - accel_offset_[1]);
	accel_tmp.setZ(accel_tmp.getZ() - accel_offset_[2]);
	gyro_tmp.setX(gyro_tmp.getX() - gyro_offset_[0]);
	gyro_tmp.setY(gyro_tmp.getY() - gyro_offset_[1]);
	gyro_tmp.setZ(gyro_tmp.getZ() - gyro_offset_[2]);

	z << accel_tmp.getX(), accel_tmp.getY(), accel_tmp.getZ(),
			 gyro_tmp.getX(), gyro_tmp.getY(), gyro_tmp.getZ();

	KF_Update(z);

	if (gyro_kf) {
		gyro_tmp.setX(X_(10) + noise[0]);
		gyro_tmp.setY(X_(12) + noise[1]);
		gyro_tmp.setZ(X_(14) + noise[2]);
	} else {
		gyro_tmp.setX(gyro_tmp.getX() + noise[0]);
		gyro_tmp.setY(gyro_tmp.getY() + noise[1]);
		gyro_tmp.setZ(gyro_tmp.getZ() + noise[2]);
	}

	if (accel_kf) {
		accel_tmp.setX(X_(2) + noise[3]);
		accel_tmp.setY(X_(5) + noise[4]);
		accel_tmp.setZ(X_(8) + noise[5]);
	} else {
		accel_tmp.setX(accel_tmp.getX() + noise[0]);
		accel_tmp.setY(accel_tmp.getY() + noise[1]);
		accel_tmp.setZ(accel_tmp.getZ() + noise[2]);
	}

	KF_Predict(dt);

	mtx_.lock();
	gyro_ = gyro_tmp;
	accel_ = accel_tmp;
	mtx_.unlock();
}
