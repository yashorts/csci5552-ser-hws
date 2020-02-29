#include <Eigen/Core>
#include <cmath>
#include <visualization/vis.h>
#include <homework/hw1/hw1.h>

#include <random>
#include <iostream>

int main() {
  Visualizer vis;
  size_t fig_LinMLE = vis.AddFigure("Linear Proj. Dist. MLE", true);
  size_t fig_NLLS = vis.AddFigure("Non-Linear Dist. LS", true);
  vis.ShowFigures();

  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<double> dist(0.0, 1.0);

  // Generate a set of points and distances
  Eigen::VectorXd G_p_R = 2.0 * Eigen::Vector2d::Random();
  size_t n_pts = 10;
  std::vector<Eigen::VectorXd> G_p_Li(n_pts);
  std::vector<double> dists(n_pts), proj_dists(n_pts);
  std::vector<double> sigmas(n_pts);
  for (size_t i = 0; i < n_pts; ++i) {
    G_p_Li[i] = 2.0 * Eigen::Vector2d::Random(); // Generate in (-2,2) range
    sigmas[i] = 0.2;
    double noise = dist(mt) * sigmas[i];
    dists[i] = (G_p_Li[i] - G_p_R).norm() + noise;
    proj_dists[i] = G_p_Li[i].normalized().transpose() * G_p_R + noise;
  }

  // Compute Estimates:
  Eigen::Vector2d G_p_R_MLE = LinearMLESolve(proj_dists, sigmas, G_p_Li);
  Eigen::Vector2d G_p_R_NLLS = NonLinearLSSolve(dists, sigmas, G_p_Li, Eigen::VectorXd::Zero(2), 0.01, 10000);

  // Write Errors and Plot:
  std::cout << "MLE Error: " << (G_p_R_MLE - G_p_R).norm() << std::endl;
  std::cout << "NLLS Error: " << (G_p_R_NLLS - G_p_R).norm() << std::endl;

  vis.Plot(fig_LinMLE, G_p_Li, "bx");
  vis.Plot(fig_NLLS, G_p_Li, "bx");

  vis.Plot(fig_LinMLE, {G_p_R}, "gx");
  vis.Plot(fig_NLLS, {G_p_R}, "gx");

  vis.Plot(fig_LinMLE, {G_p_R_MLE}, "rx");
  vis.Plot(fig_NLLS, {G_p_R_NLLS}, "rx");

  vis.PauseFigures(-1);

  return 0;
}