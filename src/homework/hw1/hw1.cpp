#include <homework/hw1/hw1.h>

Eigen::VectorXd LinearMLESolve(std::vector<double> proj_dists, // vector of distance projected onto each landmark
                               std::vector<double> sigmas, // vector of measurement noise std devs
                               std::vector<Eigen::VectorXd> G_p_Lis) { // vector of global position of each landmark
  // TODO: Use your Maximum Likelihood Estimate to solve the Weighted Linear Least Squares Problem
  return Eigen::VectorXd::Zero(2);
}

Eigen::VectorXd NonLinearLSSolve(std::vector<double> dists, // vector of distance to each landmark
                                 std::vector<double> sigmas, // vector of measurement noise std devs
                                 std::vector<Eigen::VectorXd> G_p_Lis, // vector of global position of each landmark
                                 Eigen::VectorXd G_p_R_0, // Initial guess of robot position
                                 double alpha, // Learning rate for Gauss-Newton
                                 size_t n_iter) { // Number of iterations of Gauss-Newton to run
  // TODO: Use Gauss-Newton to iteratively solve the Weighted Non-Linear Least Squares Problem
  return G_p_R_0;
}