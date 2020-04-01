#pragma once

#include <Eigen/Core>
#include <vector>


// Regular EKF functions

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot position
//   Sigma_x_t: the covariance of the prior estimate of robot position
//   u: the control taken (vector of linear and angular velocity)
//   Sigma_n: covaraince of the dynamics noise
//   dt: temporal length of the step to take
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot position at t+dt
//   Sigma_x_tpdt: covaraince of the new estimate of robot position at t+dt
void EKFPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                  Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot position
//   Sigma_x_t: the covariance of the prior estimate of robot position
//   z: The relative position measurement
//   Sigma_m: The covariance o fht emeasurement noise
//   G_p_L: The knonw global position of the landmark
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot position
//   Sigma_x_tpdt: covaraince of the new estimate of robot position
void EKFRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd z, Eigen::MatrixXd Sigma_m, Eigen::VectorXd G_p_L,
               Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);


// SLAM EKF functions

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot and landmark positions
//   Sigma_x_t: the covariance of the prior estimate of robot and landmark positions
//   u: the control taken (vector of linear and angular velocity)
//   Sigma_n: covaraince of the dynamics noise
//   dt: temporal length of the step to take
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot and lanmark positions at t+dt
//   Sigma_x_tpdt: covaraince of the new estimate of robot and landmark positions at t+dt
void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                      Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot and landmark positions
//   Sigma_x_t: the covariance of the prior estimate of robot and landmark positions
//   zs: a vector of relative position measurements to unknown landmarks
//   Sigma_ms: a vecctor of the covariance for the noise associated with each measurement
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot position
//   Sigma_x_tpdt: covaraince of the new estimate of robot position
void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs, std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);