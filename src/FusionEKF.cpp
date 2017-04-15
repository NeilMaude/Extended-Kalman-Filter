#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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

  // measurement covariance matrix - laser
  // Given covariance values
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  // Given covariance values
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
    DONE:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ <<  0,0,0,0,
          0,0,0,0,
          0,0,0,0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  float rho;
  float phi;
  float rhodot;
  float px;
  float py;
  float vx;
  float vy;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    DONE:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ <<  1,1,1,1;

    //state covariance matrix P
    // This is going to be a 4x4 with non-zeros on the diagonal, zeros elsewhere
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    //the initial transition matrix F_ 
    // Will be 4x4, start empty, updated in prediction code
    ekf_.F_ = MatrixXd(4, 4);

    // inital Q matrix gets overwritten below
    // Start with an empty matrix, set in prediction code
    ekf_.Q_ = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      /*
        Conversion of polar (R, theta) co-ordinates to cartesian
        R = distance
        theta = angle to x-axis

        x = R cos(theta)
        y = R sin(theta)

        Terminology used  the radar input file is rho (distance), phi (angle) and rhodot (velocity along angle phi)
      */

      // Pick up the radar values from the measurement_pack
      rho = measurement_pack.raw_measurements_(0);
      phi = measurement_pack.raw_measurements_(1);
      rhodot = measurement_pack.raw_measurements_(2);

      // x coordinate
      px = rho * cos(phi);
      // y coordinate
      py = rho * sin(phi);
      // x and y components of velocity
      vx = rhodot * cos(phi);
      vy = rhodot * sin(phi);

      ekf_.x_ << px,py,vx,vy;

    } 
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // Laser data is in cartesian already, so use directly
      // There is no velocity information from LIDAR, so vx, vy = 0.0
      ekf_.x_ << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1),0.0,0.0;

    }

    // save the initial timestamp for delta-t calculation
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   DONE:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use sigma_ax = 9 and sigma_ay = 9 for your Q matrix (noise_ax, noise_ay).
   */

  float dt =  (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9;
  float noise_ay = 9;


  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1,  0,
              0, 0, 0,  1;

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   DONE:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Use the Extended Kalman Filter mechanism

    rho = measurement_pack.raw_measurements_(0);
    phi = measurement_pack.raw_measurements_(1);
    rhodot = measurement_pack.raw_measurements_(2);
    
    VectorXd zRadar(3);
    zRadar << rho,phi,rhodot;

    
    Tools tools;
    // Calculate the Joacobian to use in UpdateEKF()
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(zRadar);

  } else {
    // Laser updates
    // Can use the standard Kalman Filter equations here
    VectorXd zLaser(2);
    zLaser << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1);

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(zLaser);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
