#include <homework/hw2/hw2.h>
#include <visualization/vis.h>
#include <visualization/opengl/glvis.h>

#include <random>
#include <iostream>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966
#endif


void TestEKFProp(Visualizer& vis) {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<double> dist(0.0, 1.0);

  // Prior on position
  Eigen::VectorXd x_TRUE = Eigen::Vector3d(2.0, 0.0, M_PI);
  double std_x = 0.1;
  Eigen::VectorXd x_t = x_TRUE + Eigen::Vector3d(std_x * dist(mt), std_x * dist(mt), std_x * dist(mt));
  Eigen::MatrixXd Sigma_t = std_x * std_x * Eigen::Matrix3d::Identity();

  // Noise Covariance:
  double std_n = 0.2;
  Eigen::MatrixXd Sigma_n = std_n * std_n * Eigen::Matrix2d::Identity();

  // Variables for plotting
  std::vector<Eigen::VectorXd> err_x;
  err_x.push_back(Eigen::Vector2d(0,x_t[0] - x_TRUE[0]));
  std::vector<Eigen::VectorXd> p3sig_x, m3sig_x;
  p3sig_x.push_back(Eigen::Vector2d(0,3.0 * std::sqrt(Sigma_t(0,0))));
  m3sig_x.push_back(Eigen::Vector2d(0,-3.0 * std::sqrt(Sigma_t(0,0))));
  std::vector<Eigen::VectorXd> err_y;
  err_y.push_back(Eigen::Vector2d(0,x_t[1] - x_TRUE[1]));
  std::vector<Eigen::VectorXd> p3sig_y, m3sig_y;
  p3sig_y.push_back(Eigen::Vector2d(0,3.0 * std::sqrt(Sigma_t(1,1))));
  m3sig_y.push_back(Eigen::Vector2d(0,-3.0 * std::sqrt(Sigma_t(1,1))));
  std::vector<Eigen::VectorXd> err_th;
  err_th.push_back(Eigen::Vector2d(0,std::atan2(std::sin(x_t[2] - x_TRUE[2]), std::cos(x_t[2] - x_TRUE[2]))));
  std::vector<Eigen::VectorXd> p3sig_th, m3sig_th;
  p3sig_th.push_back(Eigen::Vector2d(0,3.0 * std::sqrt(Sigma_t(2,2))));
  m3sig_th.push_back(Eigen::Vector2d(0,-3.0 * std::sqrt(Sigma_t(2,2))));

  Eigen::VectorXd u = Eigen::Vector2d(0.2, 0.5);
  double dt = 0.1;
  for (size_t i = 0; i < 200; ++i) {
    // Sample n:
    Eigen::VectorXd n = Eigen::Vector2d(std_n * dist(mt), std_n * dist(mt));

    // True Propagation (we'll assume w is not close to 0):
    Eigen::VectorXd u_tmp = n+u;
    double th_old = x_TRUE[2];
    double th_new = th_old + dt * u_tmp[1];
    x_TRUE[0] += u_tmp[0]/u_tmp[1] * (std::sin(th_new) - std::sin(th_old));
    x_TRUE[1] += u_tmp[0]/u_tmp[1] * (-std::cos(th_new) + std::cos(th_old));
    x_TRUE[2] = th_new;

    // Estimated Propagation:
    Eigen::VectorXd x_new;
    Eigen::MatrixXd Sigma_new;
    EKFPropagate(x_t, Sigma_t, u, Sigma_n, dt, x_new, Sigma_new);
    x_t = x_new;
    Sigma_t = Sigma_new;

    // Update errors:
    err_x.push_back(Eigen::Vector2d((i+1)*dt,x_t[0] - x_TRUE[0]));
    p3sig_x.push_back(Eigen::Vector2d((i+1)*dt,3.0 * std::sqrt(Sigma_t(0,0))));
    m3sig_x.push_back(Eigen::Vector2d((i+1)*dt,-3.0 * std::sqrt(Sigma_t(0,0))));
    err_y.push_back(Eigen::Vector2d((i+1)*dt,x_t[1] - x_TRUE[1]));
    p3sig_y.push_back(Eigen::Vector2d((i+1)*dt,3.0 * std::sqrt(Sigma_t(1,1))));
    m3sig_y.push_back(Eigen::Vector2d((i+1)*dt,-3.0 * std::sqrt(Sigma_t(1,1))));
    err_th.push_back(Eigen::Vector2d((i+1)*dt,std::atan2(std::sin(x_t[2] - x_TRUE[2]), std::cos(x_t[2] - x_TRUE[2]))));
    p3sig_th.push_back(Eigen::Vector2d((i+1)*dt,3.0 * std::sqrt(Sigma_t(2,2))));
    m3sig_th.push_back(Eigen::Vector2d((i+1)*dt,-3.0 * std::sqrt(Sigma_t(2,2))));
  }

  // Plot the errors:
  size_t x_err_fig = vis.AddFigure("EKFProp X error", false);
  vis.Plot(x_err_fig, p3sig_x, "b-");
  vis.Plot(x_err_fig, m3sig_x, "b-");
  vis.Plot(x_err_fig, err_x, "r-");
  size_t y_err_fig = vis.AddFigure("EKFProp Y error", false);
  vis.Plot(y_err_fig, p3sig_y, "b-");
  vis.Plot(y_err_fig, m3sig_y, "b-");
  vis.Plot(y_err_fig, err_y, "r-");
  size_t th_err_fig = vis.AddFigure("EKFProp Th error", false);
  vis.Plot(th_err_fig, p3sig_th, "b-");
  vis.Plot(th_err_fig, m3sig_th, "b-");
  vis.Plot(th_err_fig, err_th, "r-");
  vis.ShowFigures();
}

void TestEKFUpdate(Visualizer& vis) {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<double> dist(0.0, 1.0);

  // Prior on position
  Eigen::VectorXd x_TRUE = Eigen::Vector3d(2.0, 0.0, M_PI);
  double std_x = 0.4;
  Eigen::VectorXd x_t = x_TRUE + Eigen::Vector3d(std_x * dist(mt), std_x * dist(mt), std_x * dist(mt));
  Eigen::MatrixXd Sigma_t = std_x * std_x * Eigen::Matrix3d::Identity();

  // Landmark location:
  Eigen::VectorXd G_p_L = Eigen::Vector2d(2,3);

  // Noise Covariance:
  double std_m = 0.3;
  Eigen::MatrixXd Sigma_m = std_m * std_m * Eigen::Matrix2d::Identity();

  // Variables for plotting
  std::vector<Eigen::VectorXd> err_x;
  err_x.push_back(Eigen::Vector2d(0,x_t[0] - x_TRUE[0]));
  std::vector<Eigen::VectorXd> p3sig_x, m3sig_x;
  p3sig_x.push_back(Eigen::Vector2d(0,3.0 * std::sqrt(Sigma_t(0,0))));
  m3sig_x.push_back(Eigen::Vector2d(0,-3.0 * std::sqrt(Sigma_t(0,0))));
  std::vector<Eigen::VectorXd> err_y;
  err_y.push_back(Eigen::Vector2d(0,x_t[1] - x_TRUE[1]));
  std::vector<Eigen::VectorXd> p3sig_y, m3sig_y;
  p3sig_y.push_back(Eigen::Vector2d(0,3.0 * std::sqrt(Sigma_t(1,1))));
  m3sig_y.push_back(Eigen::Vector2d(0,-3.0 * std::sqrt(Sigma_t(1,1))));
  std::vector<Eigen::VectorXd> err_th;
  err_th.push_back(Eigen::Vector2d(0,std::atan2(std::sin(x_t[2] - x_TRUE[2]), std::cos(x_t[2] - x_TRUE[2]))));
  std::vector<Eigen::VectorXd> p3sig_th, m3sig_th;
  p3sig_th.push_back(Eigen::Vector2d(0,3.0 * std::sqrt(Sigma_t(2,2))));
  m3sig_th.push_back(Eigen::Vector2d(0,-3.0 * std::sqrt(Sigma_t(2,2))));

  Eigen::VectorXd u = Eigen::Vector2d(0.2, 0.5);
  double dt = 0.1;
  for (size_t i = 0; i < 200; ++i) {
    // Sample m:
    Eigen::VectorXd m = Eigen::Vector2d(std_m * dist(mt), std_m * dist(mt));

    Eigen::Matrix2d C;
    C << std::cos(x_TRUE[2]), std::sin(x_TRUE[2]),
         -std::sin(x_TRUE[2]), std::cos(x_TRUE[2]);

    Eigen::VectorXd z = C * (G_p_L - x_TRUE.head<2>()) + m;

    Eigen::VectorXd x_new;
    Eigen::MatrixXd Sigma_new;
    EKFRelPosUpdate(x_t, Sigma_t, z, Sigma_m, G_p_L, x_new, Sigma_new);
    x_t = x_new;
    Sigma_t = Sigma_new;

    // Update errors:
    err_x.push_back(Eigen::Vector2d((i+1)*dt,x_t[0] - x_TRUE[0]));
    p3sig_x.push_back(Eigen::Vector2d((i+1)*dt,3.0 * std::sqrt(Sigma_t(0,0))));
    m3sig_x.push_back(Eigen::Vector2d((i+1)*dt,-3.0 * std::sqrt(Sigma_t(0,0))));
    err_y.push_back(Eigen::Vector2d((i+1)*dt,x_t[1] - x_TRUE[1]));
    p3sig_y.push_back(Eigen::Vector2d((i+1)*dt,3.0 * std::sqrt(Sigma_t(1,1))));
    m3sig_y.push_back(Eigen::Vector2d((i+1)*dt,-3.0 * std::sqrt(Sigma_t(1,1))));
    err_th.push_back(Eigen::Vector2d((i+1)*dt,std::atan2(std::sin(x_t[2] - x_TRUE[2]), std::cos(x_t[2] - x_TRUE[2]))));
    p3sig_th.push_back(Eigen::Vector2d((i+1)*dt,3.0 * std::sqrt(Sigma_t(2,2))));
    m3sig_th.push_back(Eigen::Vector2d((i+1)*dt,-3.0 * std::sqrt(Sigma_t(2,2))));
  }

  // Plot the errors:
  size_t x_err_fig = vis.AddFigure("EKF Update X error", false);
  vis.Plot(x_err_fig, p3sig_x, "b-");
  vis.Plot(x_err_fig, m3sig_x, "b-");
  vis.Plot(x_err_fig, err_x, "r-");
  size_t y_err_fig = vis.AddFigure("EKF Update Y error", false);
  vis.Plot(y_err_fig, p3sig_y, "b-");
  vis.Plot(y_err_fig, m3sig_y, "b-");
  vis.Plot(y_err_fig, err_y, "r-");
  size_t th_err_fig = vis.AddFigure("EKF Update Th error", false);
  vis.Plot(th_err_fig, p3sig_th, "b-");
  vis.Plot(th_err_fig, m3sig_th, "b-");
  vis.Plot(th_err_fig, err_th, "r-");
  vis.ShowFigures();
}

void TestRelPosSLAM() {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<double> dist(0.0, 1.0);
  std::uniform_real_distribution<double> lm_pos_dist(-3.0, 3.0);

  std::vector<Eigen::VectorXd> landmarks(40, Eigen::Vector2d::Zero());
  // Generate Point Landmarks:
  for (size_t i = 0; i < landmarks.size(); ++i) {
    landmarks[i][0] = lm_pos_dist(mt);
    landmarks[i][1] = lm_pos_dist(mt);
  }

  // Trajectories:
  std::vector<Eigen::VectorXd> true_path;
  std::vector<Eigen::VectorXd> est_path;

  GLVisualizer vis;
  vis.SetCenterAndHeight(Eigen::Vector2d::Zero(), 6);

  // Prior on position
  Eigen::VectorXd x_TRUE = Eigen::Vector3d(-2.0, 0.0, M_PI_2);
  double std_x = 0.05;
  Eigen::VectorXd x_t = x_TRUE + Eigen::Vector3d(std_x * dist(mt), std_x * dist(mt), std_x * dist(mt));
  Eigen::MatrixXd Sigma_t = std_x * std_x * Eigen::Matrix3d::Identity();
  true_path.push_back(x_TRUE);
  est_path.push_back(x_t);

  // Noise Covariance:
  double std_n = 0.03;
  Eigen::MatrixXd Sigma_n = std_n * std_n * Eigen::Matrix2d::Identity();
  double std_m = 0.05;
  Eigen::MatrixXd Sigma_m = std_m * std_m * Eigen::Matrix2d::Identity();

  Eigen::VectorXd u = Eigen::Vector2d(0.2, -0.1);
  double dt = 0.1;
  for (size_t i = 0; i < 1000; ++i) {
    // Sample n:
    Eigen::VectorXd n = Eigen::Vector2d(std_n * dist(mt), std_n * dist(mt));

    // True Propagation (we'll assume w is not close to 0):
    Eigen::VectorXd u_tmp = n+u;
    double th_old = x_TRUE[2];
    double th_new = th_old + dt * u_tmp[1];
    x_TRUE[0] += u_tmp[0]/u_tmp[1] * (std::sin(th_new) - std::sin(th_old));
    x_TRUE[1] += u_tmp[0]/u_tmp[1] * (-std::cos(th_new) + std::cos(th_old));
    x_TRUE[2] = th_new;

    // Run an EKFSLAMPropagation step
    {
      Eigen::VectorXd x_new;
      Eigen::MatrixXd Sigma_new;
      EKFSLAMPropagate(x_t, Sigma_t, u, Sigma_n, dt, x_new, Sigma_new);
      x_t = x_new;
      Sigma_t = Sigma_new;
    }

    // Run an update every 5 iterations
    if (i%5 == 0) {
      // Compute all the landmarks within 2m
      std::vector<Eigen::VectorXd> close_z;
      std::vector<Eigen::MatrixXd> close_Sigma_m;
      for (auto& lm : landmarks) {
        if ((lm.head<2>() - x_TRUE.head<2>()).norm() < 2.0) {
          // Sample m:
          Eigen::VectorXd m = Eigen::Vector2d(std_m * dist(mt), std_m * dist(mt));

          Eigen::Matrix2d C;
          C << std::cos(x_TRUE[2]), std::sin(x_TRUE[2]),
              -std::sin(x_TRUE[2]), std::cos(x_TRUE[2]);

          close_z.push_back(C * (lm.head<2>() - x_TRUE.head<2>()) + m);
          close_Sigma_m.push_back(Sigma_m);
        }
      }

      if (close_z.size() > 0) {
        Eigen::VectorXd x_new;
        Eigen::MatrixXd Sigma_new;
        EKFSLAMRelPosUpdate(x_t, Sigma_t, close_z, close_Sigma_m, x_new, Sigma_new);
        x_t = x_new;
        Sigma_t = Sigma_new;
      }
    }

    // Draw the current state
    vis.ClearTempLines();
    // Draw the trajectory and the estimated trajectory
    true_path.push_back(x_TRUE);
    est_path.push_back(x_t);
    vis.AddTempLine(est_path, Color::RED, 1.5);
    vis.AddTempLine(true_path, Color::BLUE, 1.5);
    // Draw the uncertainty of the robot
    vis.AddTempEllipse(x_t.head<2>(), Sigma_t.topLeftCorner<2,2>(), Color::RED, 1.2);

    // Draw the uncertainty of all the landmarks in the state
    for (size_t j = 3; j < x_t.size(); j+=2) {
      vis.AddTempEllipse(x_t.segment<2>(j), Sigma_t.block<2,2>(j,j), Color::RED, 1.2);
    }

    // Draw the true landmark positions
    for (size_t j = 0; j < landmarks.size(); ++j) {
      vis.AddTempEllipse(landmarks[j].head<2>(), 0.0001 * Eigen::Matrix2d::Identity(), Color::BLUE, 2);
    }
    
    vis.UpdateLines();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  vis.PauseFigure(-1);
}

int main() {
  {
    Visualizer vis;
    // First Let's test the standard EKF equations:
    TestEKFProp(vis);
    vis.PauseFigures(1);
    TestEKFUpdate(vis);
    vis.PauseFigures(-1);
  }

  {
    // Now generate a new scene for SLAM and try to Update estimates through it
    TestRelPosSLAM();
  }
}