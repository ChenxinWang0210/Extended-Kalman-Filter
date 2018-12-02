#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  
  
  // Initial state transition matrix, will be updated according to delta_time
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ <<1,0,1,0,
  	   		0,1,0,1,
  	   		0,0,1,0,
  	   		0,0,0,1;
  
  // State covariance matrix ( The first measurement only provides position, so we have a large variance of velocity)
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
  			 0, 1, 0, 0,
  			 0, 0, 1000, 0,
  			 0, 0, 0, 1000;
  
  // laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;
  
  // radar measurement matrix (Initialize, will be updated according to position and velocity)
  Hj_ << 0.6, 0.8, 0, 0,
  		 -0.8, 0.6, 0, 0,
  		 0, 0, 0.6, 0.8;
  


    
    
  // set the acceleration noise components
    noise_ax = 9;
    noise_ay = 9;
  
  // 
    use_laser_ = true;
    use_radar_ = true;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
//     ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float radius = measurement_pack.raw_measurements_[0];
      float angle = measurement_pack.raw_measurements_[1];
      
      float px = radius*cos(angle);
      float py = radius*sin(angle);
      
      ekf_.x_ <<px, py, 0, 0;
        
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      
      ekf_.x_ <<measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      
    }
    
	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // compute the time elapsed between current and previous measurements
  float delta_t = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0; // delta_t in seconds
  
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = delta_t;
  ekf_.F_(1,3) = delta_t;
  
  // Modify process covariance matrix
  float delta_t2 = delta_t*delta_t;
  float delta_t3 = delta_t2*delta_t;
  float delta_t4 = delta_t3*delta_t;
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << delta_t4/4*noise_ax, 0, delta_t3/2*noise_ax, 0,
  			0, delta_t4/4*noise_ay, 0, delta_t3/2*noise_ay,
  			delta_t3/2*noise_ax, 0, delta_t2*noise_ax, 0,
  			0, delta_t3/2*noise_ay, 0, delta_t2*noise_ay;
  
  ekf_.Predict();
  


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  
  

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    ekf_.H_ = MatrixXd(3,4);
    ekf_.R_ = MatrixXd(3,3);
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_; 
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
    
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_ ) {
    // Laser updates
    ekf_.H_ = MatrixXd(2,4);
    ekf_.R_ = MatrixXd(2,2);
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_; 
    
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
