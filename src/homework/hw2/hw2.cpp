#include <homework/hw2/hw2.h>
#include <iostream>
#include <cmath>

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot position
//   Sigma_x_t: the covariance of the prior estimate of robot position
//   u: the control taken (vector of linear and angular velocity)
//   Sigma_n: covaraince of the dynamics noise
//   dt: temporal length of the step to take
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot position at t+dt
//   Sigma_x_tpdt: covaraince of the new estimate of robot position at t+dt
void
EKFPropagate(Eigen::VectorXd x_hat_t,
             Eigen::MatrixXd Sigma_x_t,
             Eigen::VectorXd u,
             Eigen::MatrixXd Sigma_n,
             double dt,
             Eigen::VectorXd &x_hat_tpdt,
             Eigen::MatrixXd &Sigma_x_tpdt) {
    // TODO
    double x_t = x_hat_t[0];
    double y_t = x_hat_t[1];
    double theta_t = x_hat_t[2];
    double v = u[0];
    double w = u[1];
    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    x_hat_tpdt[0] += dt * v * cos(theta_t);
    x_hat_tpdt[1] += dt * v * sin(theta_t);
    x_hat_tpdt[2] += dt * w;
    Sigma_x_tpdt = Sigma_x_t;
}

void EKFRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd z, Eigen::MatrixXd Sigma_m,
                     Eigen::VectorXd G_p_L,
                     Eigen::VectorXd &x_hat_tpdt, Eigen::MatrixXd &Sigma_x_tpdt) {
    // TODO

    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;
}


void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n,
                      double dt,
                      Eigen::VectorXd &x_hat_tpdt, Eigen::MatrixXd &Sigma_x_tpdt) {
    // TODO

    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;
}

void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs,
                         std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd &x_hat_tpdt, Eigen::MatrixXd &Sigma_x_tpdt) {
    // TODO

    // For each measurement, check if it matches any already in the state, and run an update for it.

    // For every unmatched measurement make sure it's sufficiently novel, then add to the state.

    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;
}