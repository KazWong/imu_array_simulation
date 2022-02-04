#include <imu_array_simulation/kalman_filter.h>

// Eigen Matrix default is Col Major
KalmanFilter::KalmanFilter(int _nz, int _nx, int _nu) : nz_(_nz), nx_(_nx), nu_(_nu),
		X_(new Eigen::VectorXd(_nx)),
		P_(new Eigen::MatrixXd(_nx, _nx)),
		F_(new Eigen::MatrixXd(_nx, _nx)),
		B_(new Eigen::MatrixXd(_nx, _nu)),
		H_(new Eigen::MatrixXd(_nz, _nx)),
		Q_(new Eigen::MatrixXd(_nx, _nx)),
		R_(new Eigen::MatrixXd(_nz, _nz)),
	 	K_(_nx, _nz),
		I_(nx_, nx_) {
	*X_ << Eigen::MatrixXd::Zero(_nx, 1);
	*P_ << Eigen::MatrixXd::Zero(_nx, _nx);
	*F_ << Eigen::MatrixXd::Zero(_nx, _nx);
	*B_ << Eigen::MatrixXd::Zero(_nx, _nu);
	*H_ << Eigen::MatrixXd::Zero(_nz, _nx);
	*Q_ << Eigen::MatrixXd::Zero(_nx, _nx);
	*R_ << Eigen::MatrixXd::Zero(_nz, _nz);
	K_ << Eigen::MatrixXd::Zero(_nx, _nz);
	I_ << Eigen::MatrixXd::Identity(nx_, nx_);
}

KalmanFilter::KalmanFilter(int _nz, int _nx, int _nu,
		std::shared_ptr<Eigen::VectorXd> _X,
		std::shared_ptr<Eigen::MatrixXd> _P,
		std::shared_ptr<Eigen::MatrixXd> _F,
		std::shared_ptr<Eigen::MatrixXd> _B,
		std::shared_ptr<Eigen::MatrixXd> _H,
		std::shared_ptr<Eigen::MatrixXd> _Q,
		std::shared_ptr<Eigen::MatrixXd> _R) :
	nz_(_nz), nx_(_nx), nu_(_nu),
	X_(_X), P_(_P), F_(_F), B_(_B), H_(_H), Q_(_Q), R_(_R),
	K_(_nx, _nz),
	I_(nx_, nx_)  {
		K_ << Eigen::MatrixXd::Zero(_nx, _nz);
		I_ << Eigen::MatrixXd::Identity(nx_, nx_);
}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::Init(Eigen::VectorXd _X, Eigen::MatrixXd _P, Eigen::MatrixXd _F,
		Eigen::MatrixXd _B, Eigen::MatrixXd _H, Eigen::MatrixXd _Q, Eigen::MatrixXd _R) {
	*X_ = _X;
	*P_ = _P;

	*F_ = _F;
	*B_ = _B;
	*H_ = _H;
	*Q_ = _Q;
	*R_ = _R;
}

void KalmanFilter::Predict(const Eigen::VectorXd& u) {
	*X_ = (*F_)*(*X_) + (*B_)*u;
	*P_ = (*F_)*(*P_)*(*F_).transpose() + (*F_)*(*Q_)*(*F_).transpose();

	// std::cout << "X: \n" << *X_ << "\n\n";
}

void KalmanFilter::Update(const Eigen::VectorXd& z) {
	// gain update
	Eigen::MatrixXd PpH_T = (*P_)*(*H_).transpose();
	K_ = PpH_T*( ((*H_)*PpH_T + (*R_)).inverse() );

	// measurement and state update
	Eigen::MatrixXd Y = z - (*H_)*(*X_);
	*X_ = (*X_) + (K_)*Y;

	// covariance update
	Eigen::MatrixXd I_KH = (I_ - (K_)*(*H_));
	*P_ = I_KH*(*P_)*I_KH.transpose() + (K_)*(*R_)*(K_).transpose();

	// std::cout << "K: \n" << K_ << "\n";
	// std::cout << "X: \n" << *X_ << "\n";
	// std::cout << "z: \n" << z << "\n\n";
}

Eigen::VectorXd KalmanFilter::GetEstState() const {
	return *X_;
}
