#include <homework/hw2/hw2.h>
#include <cmath>
#include <Eigen/Dense>

#include <iostream>

void EKFPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                  Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // Run the dynamics
  x_hat_tpdt = x_hat_t;
  x_hat_tpdt[0] += dt * u[0] * std::cos(x_hat_t[2]);
  x_hat_tpdt[1] += dt * u[0] * std::sin(x_hat_t[2]);
  x_hat_tpdt[2] += dt * u[1];

  // Compute the partial derivatives:
  Eigen::MatrixXd A = Eigen::Matrix3d::Identity();
  A(0,2) = -dt * u[0] * std::sin(x_hat_t[2]);
  A(1,2) = dt * u[0] * std::cos(x_hat_t[2]);
  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(3,2);
  N(0,0) = dt*std::cos(x_hat_t[2]);
  N(1,0) = dt*std::sin(x_hat_t[2]);
  N(2,1) = dt;

  // Update the covaraince matrix:
  Sigma_x_tpdt = A * Sigma_x_t * A.transpose() + N * Sigma_n * N.transpose();
}

void EKFRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd z, Eigen::MatrixXd Sigma_m, Eigen::VectorXd G_p_L,
                     Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // Compute the rotation matrix:
  Eigen::MatrixXd C_th = Eigen::Matrix2d::Zero();
  C_th << std::cos(x_hat_t[2]), -std::sin(x_hat_t[2]), std::sin(x_hat_t[2]), std::cos(x_hat_t[2]);

  // Compute the expected measurement:
  Eigen::VectorXd z_hat = C_th.transpose() * (G_p_L - x_hat_t.head<2>());

  // Compute the partial derivatives:
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,3);
  H.leftCols<2>() = -C_th.transpose();
  H(0,2) = -std::sin(x_hat_t[2]) * (G_p_L[0] - x_hat_t[0]) + std::cos(x_hat_t[2]) * (G_p_L[1] - x_hat_t[1]);
  H(1,2) = -std::cos(x_hat_t[2]) * (G_p_L[0] - x_hat_t[0]) - std::sin(x_hat_t[2]) * (G_p_L[1] - x_hat_t[1]);

  Eigen::MatrixXd M = Eigen::Matrix2d::Identity();

  // Compute S and K:
  Eigen::MatrixXd S = H * Sigma_x_t * H.transpose() + M * Sigma_m * M.transpose();
  Eigen::MatrixXd K = Sigma_x_t * H.transpose() * S.inverse();

  // Run the actual update equations:
  x_hat_tpdt = x_hat_t + K * (z - z_hat);
  // Use the Joseph form
  Sigma_x_tpdt = (Eigen::MatrixXd::Identity(3,3) - K*H) * Sigma_x_t * (Eigen::MatrixXd::Identity(3,3) - K*H).transpose() + K * M * Sigma_m * M.transpose() * K.transpose();
  // Sigma_x_tpdt = Sigma_x_t - Sigma_x_t * H.transpose() * S.inverse() * H * Sigma_x_t;
}



void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                      Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  bool has_landmarks = x_hat_t.size() > 3;
  size_t n_landmarks = (x_hat_t.size() - 3) / 2;
  // Run the dynamics
  x_hat_tpdt = x_hat_t;
  x_hat_tpdt[0] += dt * u[0] * std::cos(x_hat_t[2]);
  x_hat_tpdt[1] += dt * u[0] * std::sin(x_hat_t[2]);
  x_hat_tpdt[2] += dt * u[1];

  // Compute the partial derivatives:
  Eigen::MatrixXd A_R = Eigen::Matrix3d::Identity();
  A_R(0,2) = -dt * u[0] * std::sin(x_hat_t[2]);
  A_R(1,2) = dt * u[0] * std::cos(x_hat_t[2]);
  Eigen::MatrixXd N_R = Eigen::MatrixXd::Zero(3,2);
  N_R(0,0) = dt*std::cos(x_hat_t[2]);
  N_R(1,0) = dt*std::sin(x_hat_t[2]);
  N_R(2,1) = dt;

  // Run the covariance update:
  Sigma_x_tpdt = Sigma_x_t;
  Sigma_x_tpdt.topLeftCorner<3,3>() = A_R * Sigma_x_t.topLeftCorner<3,3>() * A_R.transpose() + N_R * Sigma_n * N_R.transpose();
  if (has_landmarks) {
    for (size_t i = 0; i < n_landmarks; ++i) {
      Sigma_x_tpdt.block<3,2>(0,3+2*i) = A_R * Sigma_x_tpdt.block<3,2>(0,3+2*i);
      Sigma_x_tpdt.block<2,3>(3+2*i,0) = Sigma_x_tpdt.block<2,3>(3+2*i,0) * A_R.transpose();
    }
  }
}

void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs, std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  size_t n_landmarks = (x_hat_t.size() - 3) / 2;

  std::vector<Eigen::VectorXd> new_zs;
  std::vector<Eigen::MatrixXd> new_Sigma_ms;
  // For each measurement, check if it matches any already in the state, and run an update for it.
  for (size_t z_idx = 0; z_idx < zs.size(); ++z_idx) {
    // Compare against every one in the state
    double best_maha_dist = 1000000000;
    size_t best_lm_idx = 0;
    Eigen::MatrixXd H_R_best, H_Li_best;
    Eigen::Vector2d z_hat_best;
    for (size_t lm_idx = 0; lm_idx < n_landmarks; ++lm_idx) {
      // Compute the rotation matrix:
      Eigen::MatrixXd C_th = Eigen::Matrix2d::Zero();
      C_th << std::cos(x_hat_t[2]), -std::sin(x_hat_t[2]), std::sin(x_hat_t[2]), std::cos(x_hat_t[2]);

      Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(2,3);
      H_R.leftCols<2>() = -C_th.transpose();
      H_R(0,2) = -std::sin(x_hat_t[2]) * (x_hat_t[3+2*lm_idx] - x_hat_t[0]) + std::cos(x_hat_t[2]) * (x_hat_t[3+2*lm_idx+1] - x_hat_t[1]);
      H_R(1,2) = -std::cos(x_hat_t[2]) * (x_hat_t[3+2*lm_idx] - x_hat_t[0]) - std::sin(x_hat_t[2]) * (x_hat_t[3+2*lm_idx+1] - x_hat_t[1]);

      Eigen::MatrixXd H_Li = C_th.transpose();

      Eigen::Vector2d lm_pos = x_hat_t.segment<2>(3+2*lm_idx);
      Eigen::VectorXd z_hat = C_th.transpose() * (lm_pos - x_hat_t.head<2>());
      Eigen::MatrixXd S = H_R * Sigma_x_t.topLeftCorner<3,3>() * H_R.transpose() + H_R * Sigma_x_t.block<3,2>(0,3+2*lm_idx) * H_Li.transpose()
                          + H_Li * Sigma_x_t.block<2,3>(3+2*lm_idx, 0) * H_R.transpose() + H_Li * Sigma_x_t.block<2,2>(3+2*lm_idx,3+2*lm_idx) * H_Li.transpose()
                          + Sigma_ms[z_idx];
      double maha_dist = std::sqrt((zs[z_idx] - z_hat).transpose() * S.inverse() * (zs[z_idx] - z_hat));
      if (maha_dist < best_maha_dist) {
        best_maha_dist = maha_dist;
        best_lm_idx = lm_idx;
        H_R_best = H_R;
        H_Li_best = H_Li;
        z_hat_best = z_hat;
      }
    }

    if (best_maha_dist > 4.2919) { // std::sqrt(18.4207)
      // Delay initialization of new features until the end, both for speed and better linearization
      new_zs.push_back(zs[z_idx]);
      new_Sigma_ms.push_back(Sigma_ms[z_idx]);
    } else if (best_maha_dist < 2.1460) { // std::sqrt(4.6052)
      // Matched to best_lm_idx, run an update
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,x_hat_t.size());
      H.leftCols(3) = H_R_best;
      H.middleCols<2>(3+2*best_lm_idx) = H_Li_best;

      Eigen::MatrixXd S = H * Sigma_x_t * H.transpose() + Sigma_ms[z_idx];
      Eigen::MatrixXd K = Sigma_x_t * H.transpose() * S.inverse();
      x_hat_t = x_hat_t + K * (zs[z_idx] - z_hat_best);
      Eigen::MatrixXd ImKH = Eigen::MatrixXd::Identity(x_hat_t.size(),x_hat_t.size()) - K*H;
      Sigma_x_t = ImKH * Sigma_x_t * ImKH.transpose() + K * Sigma_ms[z_idx] * K.transpose();
    }
  }

  // Now initialize all the new features:
  size_t new_size = x_hat_t.size() + 2 * new_zs.size();
  x_hat_tpdt = Eigen::VectorXd::Zero(new_size);
  Sigma_x_tpdt = Eigen::MatrixXd::Zero(new_size,new_size);
  // Augment x_hat_t and Sigma_x_t
  x_hat_tpdt.head(x_hat_t.size()) = x_hat_t;
  Sigma_x_tpdt.topLeftCorner(x_hat_t.size(),x_hat_t.size()) = Sigma_x_t;
  size_t current_row = x_hat_t.size();

  // Compute the rotation matrix:
  Eigen::MatrixXd C_th = Eigen::Matrix2d::Zero();
  C_th << std::cos(x_hat_tpdt[2]), -std::sin(x_hat_tpdt[2]), std::sin(x_hat_tpdt[2]), std::cos(x_hat_tpdt[2]);
  Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(2,3);
  H_R.leftCols<2>() = -C_th.transpose();
  H_R(0,2) = -std::sin(x_hat_tpdt[2]) * (-x_hat_tpdt[0]) + std::cos(x_hat_tpdt[2]) * (-x_hat_tpdt[1]);
  H_R(1,2) = -std::cos(x_hat_tpdt[2]) * (-x_hat_tpdt[0]) - std::sin(x_hat_tpdt[2]) * (-x_hat_tpdt[1]);
  for (size_t new_z_idx = 0; new_z_idx < new_zs.size(); ++new_z_idx) {
    // Update the state vector:
    x_hat_tpdt.segment<2>(current_row) = C_th * new_zs[new_z_idx] + x_hat_tpdt.head<2>();

    // Update the covaraince matrix:
    Eigen::MatrixXd H_Li = C_th.transpose();

    Eigen::MatrixXd P1 = -Sigma_x_tpdt.leftCols<3>();
    Eigen::MatrixXd P2 = H_R.transpose();

    Eigen::MatrixXd new_block = -Sigma_x_tpdt.leftCols<3>() * H_R.transpose() * H_Li;

    Sigma_x_tpdt.middleCols<2>(current_row) = new_block;
    Sigma_x_tpdt.middleRows<2>(current_row) = new_block.transpose();

    Sigma_x_tpdt.block<2,2>(current_row,current_row) = H_Li.transpose() * (H_R * Sigma_x_tpdt.topLeftCorner<3,3>() * H_R.transpose() + new_Sigma_ms[new_z_idx]) * H_Li;

    current_row += 2;
  }
}