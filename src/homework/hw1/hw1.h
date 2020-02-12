#include <Eigen/Core>
#include <vector>

Eigen::VectorXd LinearMLESolve(std::vector<double> proj_dists, std::vector<double> sigmas, std::vector<Eigen::VectorXd> G_p_Lis);

Eigen::VectorXd NonLinearLSSolve(std::vector<double> dists, std::vector<double> sigmas, std::vector<Eigen::VectorXd> G_p_Lis, Eigen::VectorXd G_p_R_0, double alpha, size_t n_iter);