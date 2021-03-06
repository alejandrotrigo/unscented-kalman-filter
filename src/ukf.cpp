#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // Set initialization to false initially
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
    
  previous_timestamp_ = 0;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  // Weights of sigma points
  weights_ = VectorXd(n_aug_*2 + 1);
  
  weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i=1; i<n_aug_*2+1; i++){
      weights_(i) = 1/(2*(lambda_+n_aug_));
  }

  
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
  
  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  
  if (!is_initialized_) {

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
            
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_d = measurement_pack.raw_measurements_(2);
      
      double px = 0;  //rho * cos(phi);
      double py = 0; //rho * sin(phi);
      double vx = rho_d * cos(phi);
      double vy = rho_d * sin(phi);
      
      x_ << px, py, sqrt(vx*vx + vy*vy), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state. If Laser data then there is no velocity
      */
      x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0, 0;
    }
   
    if (fabs(x_(0)) < 0.00001 and fabs(x_(1)) < 0.00001){
		x_(0) = 0.00001;
		x_(1) = 0.00001;
	}
    
    //initial covariance matrix 
    P_ << 1, 0, 0, 0, 0,
			   0, 1, 0, 0, 0,
			   0, 0, 1, 0, 0,
			   0, 0, 0, 1, 0,
			   0, 0, 0, 0, 1;
			   
	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
    
  }
  
  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  
  while (dt > 0.1){
  
	double deltat = 0.05;
	Prediction(deltat);
	dt -= deltat;
 }

  Prediction(dt);
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
	  
	  VectorXd z_pred = VectorXd(3);
	  MatrixXd S = MatrixXd(3,3);
	  MatrixXd Tc = MatrixXd(n_x_,3);
	  
	  z_pred.fill(0.0);
	  S.fill(0.0);
	  Tc.fill(0.0);
	  
	  PredictRadarMeasurements(z_pred, S, Tc);
	  
	  UpdateRadar(measurement_pack, z_pred, Tc, S);
	  
	  previous_timestamp_ = measurement_pack.timestamp_;
	  
  }else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
	  
	  VectorXd z_pred = VectorXd(2);
	  MatrixXd S = MatrixXd(2,2);
	  MatrixXd Tc = MatrixXd(5,2);
	  
	  z_pred.fill(0.0);
	  S.fill(0.0);
	  Tc.fill(0.0);
	  
	  PredictLidarMeasurements(z_pred, S, Tc);
	  
	  UpdateLidar(measurement_pack, z_pred, Tc, S);
	  
	  previous_timestamp_ = measurement_pack.timestamp_;
  }
  
  return;
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  MatrixXd Xsig_aug = MatrixXd(2*n_aug_+1, n_aug_);
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2*n_aug_+1);
  
  GenerateSigmaPoints(Xsig_aug);
  PredictSigmaPoints(Xsig_aug, delta_t, Xsig_pred);
  
  VectorXd x_out_ = VectorXd(n_x_);
  MatrixXd P_out_ = MatrixXd(n_x_, n_x_);
  
  PredictMeanCovariance(Xsig_pred, x_out_, P_out_);
  
  x_ = x_out_;
  P_ = P_out_;
  Xsig_pred_ = Xsig_pred;
  
  return;
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package, VectorXd &z_pred, MatrixXd &Tc, MatrixXd &S) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
  
  // Kalman Gain
  
  MatrixXd K = MatrixXd(n_x_, 2);
  K = Tc * S.inverse();
  
  // Residual
  VectorXd y = VectorXd(2);
  y = z - z_pred;
  
  // Update state mean and covariance matrix
  x_ = x_ + K * y;
  P_ = P_- K * S * K.transpose();
  
  //NIS
  NIS_laser_ = y.transpose() * S.inverse() * y; 
  
  return;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package, VectorXd &z_pred, MatrixXd &Tc, MatrixXd &S) {
  /**

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  
  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);
  
  
  //Kalman Gain
  MatrixXd K = MatrixXd(n_x_, 3);
  K = Tc * S.inverse();

  //Residual 
  VectorXd y = VectorXd(3);
  y = z -z_pred;
  
  
  //Angle normalization
  if(y(1) < -M_PI){
	y(1) = M_PI + std::fmod(y(1) + M_PI, M_PI + M_PI);
  }else if(y(1) > M_PI){
	y(1) = std::fmod(y(1) + M_PI, M_PI + M_PI) - M_PI;
  }
  

  //update state mean and covariance matrix
  x_ = x_ + K * y;
  P_ = P_ - K * S * K.transpose();
  
  //NIS
  NIS_radar_ = y.transpose() * S.inverse() * y;
  
  return;
  
}

void UKF::GenerateSigmaPoints(MatrixXd &Xsig_out){
	
	VectorXd x_aug = VectorXd(7);
    
    MatrixXd P_aug = MatrixXd(7,7);
    
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    
    MatrixXd Q = MatrixXd(2,2);
	Q << std_a_*std_a_,0,0,std_yawdd_*std_yawdd_;
	
	P_aug.bottomRightCorner(2,2) = Q;
	
	MatrixXd A = P_aug.llt().matrixL();
	
	Xsig_aug.col(0) = x_aug;
	
	for(int i=0; i<n_aug_; i++){
      Xsig_aug.col(i+1) = x_aug + (A.col(i) * sqrt(lambda_ + n_aug_));
      Xsig_aug.col(i+n_aug_+1) = x_aug - (A.col(i) * sqrt(lambda_ + n_aug_));
	}
	 
	Xsig_out = Xsig_aug;
	
	return;
	
}

void UKF::PredictSigmaPoints(MatrixXd &Xsig_aug, double delta_t, MatrixXd &Xsig_out){
	
	MatrixXd Xsig_pred = MatrixXd(n_x_, 2*n_aug_+1);
	
	//predict sigma points
  for(int i = 0; i<2*n_aug_+1; i++){
      
      double px = Xsig_aug(0,i);
      double py = Xsig_aug(1,i);
      double v = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yawd = Xsig_aug(4,i);
      double nu_a = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);
      
      double px_p,py_p;
      //avoid division by zero
      if(fabs(yawd) > 0.001){
          px_p = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
          py_p = py + v/yawd * (-cos(yaw + yawd*delta_t) + cos(yaw));
      }else{
          px_p = px + v * (cos(yaw)) * delta_t;
          py_p = py + v * (sin(yaw)) * delta_t;
      }
      
      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;
      
      //add noise
      px_p = px_p + 0.5*nu_a*delta_t*delta_t*cos(yaw);
      py_p = py_p + 0.5*nu_a*delta_t*delta_t*sin(yaw);
      v_p = v_p + delta_t*nu_a;
      yaw_p = yaw_p + 0.5*delta_t*delta_t*nu_yawdd;
      yawd_p = yawd_p + delta_t * nu_yawdd;
      
      Xsig_pred(0,i) = px_p;
      Xsig_pred(1,i) = py_p;
      Xsig_pred(2,i) = v_p;
      Xsig_pred(3,i) = yaw_p;
      Xsig_pred(4,i) = yawd_p;
      
      
  }
  
  Xsig_out = Xsig_pred;
  
  return;
	
}

void UKF::PredictMeanCovariance(MatrixXd &Xsig_pred, VectorXd &x_out, MatrixXd &P_out){
	
  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predict state mean
  x.fill(0.0);
  for(int i=0; i<n_aug_*2+1; i++){
      x = x + weights_(i)*Xsig_pred.col(i); 
  }
  
  //predict state covariance matrix
  P.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
      VectorXd diff = Xsig_pred.col(i) - x;
      
      //angle normalization
      if(diff(3) < -M_PI){
		diff(3) = M_PI + std::fmod(diff(3) + M_PI, M_PI + M_PI);
	  }else if(diff(3) > M_PI){
		diff(3) = std::fmod(diff(3) + M_PI, M_PI + M_PI) - M_PI;
	  }
    //while (diff(3)> M_PI) diff(3)-=2.*M_PI;
	//while (diff(3)<-M_PI) diff(3)+=2.*M_PI;
      
      P = P + weights_(i)*diff*diff.transpose();
  }
  
  //write result
  x_out = x;
  P_out = P;
  
  return;
	
}

void UKF::PredictRadarMeasurements(VectorXd &z_out, MatrixXd &S_out, MatrixXd &Tc_out){
  
  //Create matrix for measurement space
  MatrixXd Zsig = MatrixXd(3, n_aug_ * 2 + 1);
	
  //Transform the sigma points into measurement space
  for(int i = 0; i < 2 * n_aug_ +1; i++){
	
	double p_x = Xsig_pred_(0,i);
	double p_y = Xsig_pred_(1,i);
	double v = Xsig_pred_(2,i);
	double yaw = Xsig_pred_(3,i);
	
	 // Avoid division by zero
    if(fabs(p_x) <= 0.0001){
            p_x = 0.0001;
    }
    if(fabs(p_y) <= 0.0001){
            p_y = 0.0001;
	}
	
	double vx = v * cos(yaw);
	double vy = v * sin(yaw);
	
	double rho = sqrt(p_x*p_x + p_y*p_y);
	double theta = atan2(p_y, p_x);
	
	if (rho < 0.0001){
		rho = 0.0001;
	}
	
	double rho_d = (p_x * vx + p_y * vy) / rho;
	
	
	Zsig(0,i) = rho;
	Zsig(1,i) = theta;
	Zsig(2,i) = rho_d;
	
  }//End for
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
	
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
	
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(3,3);
  S.fill(0.0);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
	
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
	//residual
	VectorXd z_diff = Zsig.col(i) - z_pred;

	//angle normalization
	if(z_diff(1) < -M_PI){
		z_diff(1) = M_PI + std::fmod(z_diff(1) + M_PI, M_PI + M_PI);
	}else if(z_diff(1) > M_PI){
		z_diff(1) = std::fmod(z_diff(1) + M_PI, M_PI + M_PI) - M_PI;
	}
		
	S = S + weights_(i) * z_diff * z_diff.transpose();

	// state difference
	VectorXd x_diff = Xsig_pred_.col(i) - x_;
	//angle normalization
	if(x_diff(3) < -M_PI){
		x_diff(3) = M_PI + std::fmod(x_diff(3) + M_PI, M_PI + M_PI);
	}else if(x_diff(3) > M_PI){
		x_diff(3) = std::fmod(x_diff(3) + M_PI, M_PI + M_PI) - M_PI;
	}

	Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

  }
	
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(3,3);
  R <<    pow(std_radr_,2), 0, 0,
          0, pow(std_radphi_,2), 0,
          0, 0, pow(std_radrd_,2);
  S = S + R;

  //write result
  z_out = z_pred;
  S_out = S;
  Tc_out = Tc;

  return;
	  
}

void UKF::PredictLidarMeasurements(VectorXd &z_out, MatrixXd &S_out, MatrixXd &Tc_out){

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    Zsig(0,i) = Xsig_pred_(0,i);
    Zsig(1,i) = Xsig_pred_(1,i);
    
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(2);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(2,2);
  S.fill(0.0);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(2,2);
  R <<    pow(std_laspx_,2), 0,
          0, pow(std_laspy_,2);
  S = S + R;

  //write result
  z_out = z_pred;
  S_out = S;
  Tc_out = Tc;
	
  return;
}
