#include <homework/hw2/hw2.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

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
    Eigen::Matrix<double, 3, 3> A;
    A << 1, 0, -dt * v * sin(theta_t),
            0, 1, dt * v * cos(theta_t),
            0, 0, 1;
    Eigen::Matrix<double, 3, 2> N;
    N << dt * cos(theta_t), 0,
            dt * sin(theta_t), 0,
            0, dt;
    Sigma_x_tpdt = A * Sigma_x_t * A.transpose() + N * Sigma_n * N.transpose();
}

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot position
//   Sigma_x_t: the covariance of the prior estimate of robot position
//   z: The relative position measurement
//   Sigma_m: The covariance o fht emeasurement noise
//   G_p_L: The knonw global position of the landmark
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot position
//   Sigma_x_tpdt: covaraince of the new estimate of robot position
void EKFRelPosUpdate(Eigen::VectorXd x_hat_t,
                     Eigen::MatrixXd Sigma_x_t,
                     Eigen::VectorXd z,
                     Eigen::MatrixXd Sigma_m,
                     Eigen::VectorXd G_p_L,
                     Eigen::VectorXd &x_hat_tpdt,
                     Eigen::MatrixXd &Sigma_x_tpdt) {
    double x_R_t = x_hat_t[0];
    double y_R_t = x_hat_t[1];
    double theta_t = x_hat_t[2];

    double x_L_t = G_p_L[0];
    double y_L_t = G_p_L[1];

    Eigen::Matrix<double, 2, 3> H;
    H << -cos(theta_t), -sin(theta_t), -(x_L_t - x_R_t) * sin(theta_t) + (y_L_t - y_R_t) * cos(theta_t),
            sin(theta_t), -cos(theta_t), -(x_L_t - x_R_t) * cos(theta_t) - (y_L_t - y_R_t) * sin(theta_t);
    Eigen::Matrix<double, 2, 2> M;
    M << 1, 0,
            0, 1;
    Eigen::Matrix<double, 2, 2> S = H * Sigma_x_t * H.transpose() + M * Sigma_m * M.transpose();
    Eigen::Matrix<double, 3, 2> K = Sigma_x_t * H.transpose() * S.inverse();
    Eigen::Matrix<double, 2, 2> C_G_theta_R;
    C_G_theta_R << cos(theta_t), -sin(theta_t),
            sin(theta_t), cos(theta_t);
    Eigen::Matrix<double, 2, 1> G_p_R;
    G_p_R << x_R_t,
            y_R_t;
    Eigen::Matrix<double, 2, 1> h_x_hat_0 = C_G_theta_R.transpose() * (G_p_L - G_p_R);
    Eigen::Matrix<double, 3, 3> I;
    I << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    x_hat_tpdt += K * (z - h_x_hat_0);
    Sigma_x_tpdt = (I - K * H) * Sigma_x_t * (I - K * H).transpose() + K * M * Sigma_m * M.transpose() * K.transpose();
}

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot and landmark positions
//   Sigma_x_t: the covariance of the prior estimate of robot and landmark positions
//   u: the control taken (vector of linear and angular velocity)
//   Sigma_n: covaraince of the dynamics noise
//   dt: temporal length of the step to take
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot and lanmark positions at t+dt
//   Sigma_x_tpdt: covaraince of the new estimate of robot and landmark positions at t+dt
void EKFSLAMPropagate(Eigen::VectorXd x_hat_t,
                      Eigen::MatrixXd Sigma_x_t,
                      Eigen::VectorXd u,
                      Eigen::MatrixXd Sigma_n,
                      double dt,
                      Eigen::VectorXd &x_hat_tpdt,
                      Eigen::MatrixXd &Sigma_x_tpdt) {
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
    Eigen::Matrix<double, 3, 3> A;
    A << 1, 0, -dt * v * sin(theta_t),
            0, 1, dt * v * cos(theta_t),
            0, 0, 1;
    Eigen::Matrix<double, 3, 2> N;
    N << dt * cos(theta_t), 0,
            dt * sin(theta_t), 0,
            0, dt;
    // Sigma_RR_new
    Eigen::Matrix<double, 3, 3> Sigma_RR = Sigma_x_tpdt.block<3, 3>(0, 0);
    Sigma_x_tpdt.block<3, 3>(0, 0) = A * Sigma_RR * A.transpose() + N * Sigma_n * N.transpose();
    // Sigma_R_Li_new
    for (long col = 3; col < Sigma_x_tpdt.cols(); col = col + 2) {
        Sigma_x_tpdt.block<3, 2>(0, col) = A * Sigma_x_tpdt.block<3, 2>(0, col);
    }
    // Sigma_Li_R_new
    for (long row = 3; row < Sigma_x_tpdt.rows(); row = row + 2) {
        Sigma_x_tpdt.block<2, 3>(row, 0) = Sigma_x_tpdt.block<2, 3>(row, 0) * A.transpose();
    }
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