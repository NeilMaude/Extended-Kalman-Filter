#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

}

void KalmanFilter::Predict() {
  /**
  DONE:
    * predict the state
  */
  // Kalman Filter Equations
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  DONE:
    * update the state by using Kalman Filter equations
  */
  // Standard Kalman Filter Equations
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  // Calculate the new estimate
  x_ = x_ + (K * y);
  // Need an identity matrix of the correct size
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // Calculate new prediction P_
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  DONE:
    * update the state by using Extended Kalman Filter equations
  */
  // Extended Kalman Filter Equations
  float rho_pred    =  pow(pow(x_[0],2) + pow(x_[1],2),0.5);
  float phi_pred    =  0.0;
  if (fabs(x_[0]) > 0.01) {
    phi_pred  = atan2(x_[1],x_[0]);
  }
  float rhodot_pred = 0.0;
  if (fabs(rho_pred) > 0.01) {
    rhodot_pred = (x_[0]*x_[2] + x_[1]*x_[3]) / rho_pred;
  }
  VectorXd z_pred(3);
  z_pred << rho_pred, phi_pred, rhodot_pred;

  VectorXd y = z - z_pred;  
  MatrixXd Ht = H_.transpose();
  
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // Calculate the new estimate
  x_ = x_ + (K * y);
  // Create identity matrix of required size
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // Calculate new estimate for P_
  P_ = (I - K * H_) * P_;

}
