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

using namespace std;
using namespace Eigen;

// Inputs:
//   x_hat_t: the mean of the prior estimate of robot and landmark positions
//   Sigma_x_t: the covariance of the prior estimate of robot and landmark positions
//   zs: a vector of relative position measurements to unknown landmarks
//   Sigma_ms: a vecctor of the covariance for the noise associated with each measurement
// Outputs:
//   x_hat_tpdt: mean of the new estimate of robot position
//   Sigma_x_tpdt: covaraince of the new estimate of robot position
void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t,
                         Eigen::MatrixXd Sigma_x_t,
                         std::vector<Eigen::VectorXd> zs,
                         std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd &x_hat_tpdt,
                         Eigen::MatrixXd &Sigma_x_tpdt) {
    // For each measurement, check if it matches any already in the state, and run an update for it.
    // For every unmatched measurement make sure it's sufficiently novel, then add to the state.
    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;

    double KNOWN_LANDMARK_THRESHOLD = 10;

    for (size_t i = 0; i < zs.size(); ++i) {
        // For each measurement
        VectorXd z = zs[i];
        MatrixXd Sigma_m = Sigma_ms[i];

        // Best match quantities
        double min_distance = KNOWN_LANDMARK_THRESHOLD + 1;
        MatrixXd best_H;
        Matrix<double, 2, 1> best_h_x_hat_0;
        Matrix<double, 2, 2> best_S;

        // For each landmark check the Mahalanobis distance
        int no_landmarks = (x_hat_tpdt.size() - 3) / 2;
        std::cout << no_landmarks << " landmarks" << std::endl;
        for (int j = 3; j < x_hat_tpdt.size(); j = j + 2) {
            double x_R_t = x_hat_tpdt(0, 0);
            double y_R_t = x_hat_tpdt(1, 0);
            double theta_t = x_hat_tpdt(2, 0);
            Matrix<double, 2, 1> G_p_R;
            G_p_R << x_R_t,
                    y_R_t;
            Matrix<double, 2, 2> H_L_new;
            H_L_new << cos(theta_t), sin(theta_t),
                    -sin(theta_t), cos(theta_t);
            double x_L_t = x_hat_tpdt(j, 0);
            double y_L_t = x_hat_tpdt(j + 1, 0);
            Matrix<double, 2, 1> G_p_L;
            G_p_L << x_L_t,
                    y_L_t;

            Matrix<double, 2, 3> H_R;
            H_R << -cos(theta_t), -sin(theta_t), -sin(theta_t) * (x_L_t - x_R_t) + cos(theta_t) * (y_L_t - y_R_t),
                    sin(theta_t), -cos(theta_t), -cos(theta_t) * (x_L_t - x_R_t) - sin(theta_t) * (y_L_t - y_R_t);

            MatrixXd H = MatrixXd::Zero(2, x_hat_tpdt.size());
            H.block<2, 3>(0, 0) = H_R;
            H.block<2, 2>(0, j) = H_L_new;

            Matrix<double, 2, 2> S = H * Sigma_x_tpdt * H.transpose() + Sigma_m;

            Matrix<double, 2, 1> h_x_hat_0 = H_L_new * (G_p_L - G_p_R);

            double distance = (z - h_x_hat_0).transpose() * S.inverse() * (z - h_x_hat_0);

            // Track the most likely landmark
            if (distance < min_distance) {
                min_distance = distance;
                best_H = H;
                best_h_x_hat_0 = h_x_hat_0;
                best_S = S;
            }
        }

        // If looks like a landmark then do a regular update
        if (min_distance <= KNOWN_LANDMARK_THRESHOLD) {
            MatrixXd K = Sigma_x_tpdt * best_H.transpose() * best_S.inverse();
            MatrixXd I = MatrixXd::Identity(x_hat_tpdt.size(), x_hat_tpdt.size());

            // Note that these we passed by reference, so to return, just set them
            x_hat_tpdt = x_hat_tpdt + K * (z - best_h_x_hat_0);
            Sigma_x_tpdt = (I - K * best_H) * Sigma_x_tpdt * (I - K * best_H).transpose() +
                           K * Sigma_m * K.transpose();
            continue;
        }

        // If looks like no landmark seen until now augment SLAM state with the landmark information
        if (min_distance > KNOWN_LANDMARK_THRESHOLD) {
            double x_R_t = x_hat_tpdt(0, 0);
            double y_R_t = x_hat_tpdt(1, 0);
            double theta_t = x_hat_tpdt(2, 0);
            Matrix<double, 2, 1> G_p_R;
            G_p_R << x_R_t,
                    y_R_t;
            Matrix<double, 2, 2> H_L_new;
            H_L_new << cos(theta_t), sin(theta_t),
                    -sin(theta_t), cos(theta_t);

            // Expected value
            // Copy previous state
            MatrixXd tmp_x_hat_tpdt = x_hat_tpdt;
            x_hat_tpdt = MatrixXd::Zero(x_hat_tpdt.size() + 2, 1);
            for (int t = 0; t < tmp_x_hat_tpdt.size(); ++t) {
                x_hat_tpdt(t, 0) = tmp_x_hat_tpdt(t, 0);
            }
            // Add new landmark estimate
            Matrix<double, 2, 1> h_x_hat_0 = H_L_new * (Matrix<double, 2, 1>::Zero() - G_p_R);
            x_hat_tpdt.block<2, 1>(tmp_x_hat_tpdt.size(), 0) = H_L_new.inverse() * (z - h_x_hat_0);

            // Covariance
            // Copy previous state
            MatrixXd tmp_Sigma_x_tpdt = Sigma_x_tpdt;
            Sigma_x_tpdt = MatrixXd::Zero(Sigma_x_tpdt.rows() + 2, Sigma_x_tpdt.cols() + 2);
            for (int t = 0; t < tmp_Sigma_x_tpdt.rows(); ++t) {
                for (int s = 0; s < tmp_Sigma_x_tpdt.cols(); ++s) {
                    Sigma_x_tpdt(t, s) = tmp_Sigma_x_tpdt(t, s);
                }
            }
            // Augmentation
            Matrix<double, 2, 3> H_R;
            H_R << -cos(theta_t), -sin(theta_t), -(0 - x_R_t) * sin(theta_t) + (0 - y_R_t) * cos(theta_t),
                    sin(theta_t), -cos(theta_t), -(0 - x_R_t) * cos(theta_t) - (0 - y_R_t) * sin(theta_t);
            MatrixXd H_L_new_inv = H_L_new.inverse();
            // Top right block
            Sigma_x_tpdt.block<3, 2>(0, tmp_Sigma_x_tpdt.cols()) =
                    -tmp_Sigma_x_tpdt.block<3, 3>(0, 0) * H_R.transpose() * H_L_new_inv.transpose();
            // Bottom left block
            Sigma_x_tpdt.block<2, 3>(tmp_Sigma_x_tpdt.rows(), 0) =
                    -H_L_new_inv * H_R * tmp_Sigma_x_tpdt.block<3, 3>(0, 0);
            // Bottom right block
            Sigma_x_tpdt.block<2, 2>(tmp_Sigma_x_tpdt.rows(), tmp_Sigma_x_tpdt.cols())
                    = H_L_new_inv
                      * (
                              H_R * tmp_Sigma_x_tpdt.block<3, 3>(0, 0) * H_R.transpose()
                              + Sigma_m
                      )
                      * H_L_new_inv.transpose();
            // Bottom row
            for (int col = 3; col < tmp_Sigma_x_tpdt.cols(); col = col + 2) {
                Sigma_x_tpdt.block<2, 2>(tmp_Sigma_x_tpdt.rows(), col) =
                        -H_L_new_inv * H_R * tmp_Sigma_x_tpdt.block<3, 2>(0, col);
            }
            // Right row
            for (int row = 3; row < tmp_Sigma_x_tpdt.rows(); row = row + 2) {
                Sigma_x_tpdt.block<2, 2>(row, tmp_Sigma_x_tpdt.cols()) =
                        -tmp_Sigma_x_tpdt.block<2, 3>(row, 0) * H_R.transpose() * H_L_new_inv.transpose();
            }
            continue;
        }
    }
}
