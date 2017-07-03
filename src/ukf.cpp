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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 6;

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

  
  // Weights of sigma points
  weights_ = VectorXd(15);

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  

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

    
    // first measurement
    cout << "UKF: " << endl;
    x_ = VectorXd(5);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
            
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_d = measurement_pack.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_d * cos(phi);
      float vy = rho_d * sin(phi);
      
      x_ << px, py, sqrt(vx*vx + vy*vy), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state. If Laser data then there is no velocity
      */
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    }
   
    if (fabs(x_(0)) < 0.00001 and fabs(x_(1)) < 0.00001){
		x_(0) = 0.00001;
		x_(1) = 0.00001;
	}
    
    //initial covariance matrix 
    P_ = MatrixXd(5,5);
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
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out){
	
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
	
	std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
	*Xsig_out = Xsig_aug;
	
}

void PredictSigmaPoints(){
	
	/**
	TODO
	* 
	**/
	
}

void PredictMeanCovariance(){
	
	/**
	TODO
	*
	**/
	
}

void PredictMeasurement(){
	
	/**
	TODO
	*
	**/
	
}
