#include <homework/hw2/hw2.h>

void EKFPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                  Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // TODO


  // Note that these we passed by reference, so to return, just set them
  x_hat_tpdt = x_hat_t;
  Sigma_x_tpdt = Sigma_x_t;
}

void EKFRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd z, Eigen::MatrixXd Sigma_m, Eigen::VectorXd G_p_L,
                     Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // TODO

  // Note that these we passed by reference, so to return, just set them
  x_hat_tpdt = x_hat_t;
  Sigma_x_tpdt = Sigma_x_t;
}



void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                      Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // TODO

  // Note that these we passed by reference, so to return, just set them
  x_hat_tpdt = x_hat_t;
  Sigma_x_tpdt = Sigma_x_t;
}

void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs, std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // TODO

  // For each measurement, check if it matches any already in the state, and run an update for it.

  // For every unmatched measurement make sure it's sufficiently novel, then add to the state.

  // Note that these we passed by reference, so to return, just set them
  x_hat_tpdt = x_hat_t;
  Sigma_x_tpdt = Sigma_x_t;
}