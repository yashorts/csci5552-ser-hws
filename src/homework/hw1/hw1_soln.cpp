#include <homework/hw1/hw1.h>
#include <Eigen/Dense> // Needs to be included to call .inverse()

#include <iostream>

Eigen::VectorXd LinearMLESolve(std::vector<double> proj_dists, // vector of distance projected onto each landmark
                               std::vector<double> sigmas, // vector of measurement noise std devs
                               std::vector<Eigen::VectorXd> G_p_Lis) { // vector of global position of each landmark
  // TODO: Use your Maximum Likelihood Estimate to solve the Weighted Linear Least Squares Problem
  int n_dims = G_p_Lis[0].size();

  Eigen::MatrixXd HSH_sum = Eigen::MatrixXd::Zero(n_dims, n_dims);
  Eigen::VectorXd HSz_sum = Eigen::VectorXd::Zero(n_dims);

  for (size_t i = 0; i < proj_dists.size(); ++i) {
    Eigen::VectorXd H_i = G_p_Lis[i].normalized();

    HSH_sum += H_i * 1.0 / (sigmas[i]*sigmas[i]) * H_i.transpose();
    HSz_sum += H_i * 1.0 / (sigmas[i]*sigmas[i]) * proj_dists[i];
  }

  Eigen::VectorXd est_pos = HSH_sum.inverse() * HSz_sum;

  return est_pos;
}

Eigen::VectorXd NonLinearLSSolve(std::vector<double> dists, // vector of distance to each landmark
                                 std::vector<double> sigmas, // vector of measurement noise std devs
                                 std::vector<Eigen::VectorXd> G_p_Lis, // vector of global position of each landmark
                                 Eigen::VectorXd G_p_R_0, // Initial guess of robot position
                                 double alpha, // Learning rate for Gauss-Newton
                                 size_t n_iter) { // Number of iterations of Gauss-Newton to run
  // TODO: Use Gauss-Newton to iteratively solve the Weighted Non-Linear Least Squares Problem
  Eigen::VectorXd G_p_R_i = G_p_R_0;
  int n_dims = G_p_Lis[0].size();

  for (size_t iter = 0; iter < n_iter; ++iter) {
    Eigen::MatrixXd HSH_sum = Eigen::MatrixXd::Zero(n_dims, n_dims);
    Eigen::VectorXd HSr_sum = Eigen::VectorXd::Zero(n_dims);
    for (size_t i =0 ; i < dists.size(); ++i) {
      Eigen::Vector2d H_i = (G_p_Lis[i] - G_p_R_i).normalized(); 
      double r_i = dists[i] - (G_p_Lis[i] - G_p_R_i).norm();
      HSH_sum += H_i * 1.0 / (sigmas[i]*sigmas[i]) * H_i.transpose();
      HSr_sum += H_i * 1.0 / (sigmas[i]*sigmas[i]) * r_i;
    }

    G_p_R_i = G_p_R_i - alpha * HSH_sum.inverse() * HSr_sum;
  }

  return G_p_R_i;
}