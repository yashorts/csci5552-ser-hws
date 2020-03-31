#pragma once

#include <Eigen/Core>
#include <vector>


// Regular EKF functions
void EKFPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                  Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);

void EKFRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd z, Eigen::MatrixXd Sigma_m, Eigen::VectorXd G_p_L,
               Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);


// SLAM EKF functions
void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                      Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);

void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs, std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);