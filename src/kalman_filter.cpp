#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict() {
  /**
   * TODO/DONE: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO/DONE: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  UpdateEstimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO/DONE: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // Calculate h(x')
  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float phi = atan2(py, px);
  float rhoDot = (px * vx + py * vy) / rho;
  VectorXd h = VectorXd(3);
  h << rho, phi, rhoDot;

  // Calculate y
  VectorXd y = z - h;
  y(1) = NormalizeAngle(y(1));

  UpdateEstimate(y);
}

void KalmanFilter::UpdateEstimate(VectorXd &y) {
  MatrixXd Ht = H_.transpose();

  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

float KalmanFilter::NormalizeAngle(float angle) {
  float normalizedAngle = angle;
  if (angle > M_PI) {
    normalizedAngle = angle - M_PI * 2.0;
  } else if (angle < -M_PI) {
    normalizedAngle = angle + M_PI * 2.0;
  }

  return normalizedAngle;
}


