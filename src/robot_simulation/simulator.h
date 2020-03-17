#pragma once

#define _USE_MATH_DEFINES // For Windows: Gives M_PI, etc.

#include <robot_simulation/line_segment.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include <thread>
#include <random>

struct LaserScanData {
  std::vector<double> scan;
  std::chrono::high_resolution_clock::time_point scan_time;
};

struct OdometryData {
  Eigen::Vector2d odom;
  std::chrono::high_resolution_clock::time_point odom_time;
};

class Simulator {
public:
  // Laser Scanner Parameters:
  static constexpr size_t n_lasers = 181;
  static constexpr double min_th = -M_PI_2;
  static constexpr double max_th = M_PI_2;
  static constexpr double max_laser_dist = 5.0;
  static constexpr double laser_dist_overdist_val = 6.0;

  // Robot Parameters:
  double robot_radius; //TODO: Maybe load this from map file?

  Simulator(std::string map_file);

  std::vector<std::vector<Eigen::VectorXd>> GetLines();
  Eigen::Vector3d GetTruePose();

  LaserScanData GetLaserScan();
  OdometryData GetOdometry();
  void SendControl(Eigen::Vector2d ctrl);
  bool IsAlive();

  ~Simulator();

private:
  std::vector<LineSegment> line_features;
  Eigen::Vector3d true_pose;

  std::thread main_loop;
  bool running = true;

  // Random Distributions
  std::mt19937 mt;
  double th_err_perc = 0.5;
  std::uniform_real_distribution<double> laser_th_err;
  double d_err = 0.05;
  std::uniform_real_distribution<double> laser_d_err;
  double v_err = 0.1;
  std::uniform_real_distribution<double> odom_v_err;
  double w_err = 0.1;
  std::uniform_real_distribution<double> odom_w_err;

  // Laser Scanner Parameters:
  std::mutex laser_scan_lock;
  int laser_scan_freq = 10;  // How many iterations to wait between updates
  LaserScanData current_laser_scan;


  // Robot Parameters:
  double max_v_accel = 0.5;
  double max_w_accel = 0.5;
  std::mutex odometry_lock;
  std::mutex control_lock;
  Eigen::Vector2d goal_control = Eigen::Vector2d::Zero();
  Eigen::Vector2d current_control = Eigen::Vector2d::Zero();
  OdometryData current_odom_data;
  int control_freq = 1; // How many iterations to wait between updates

  std::vector<double> ComputeLaserScan();
  Eigen::Vector3d ContDynamics(const Eigen::Vector3d& x, const Eigen::Vector2d& u);
  void UpdateCurrentControl(double dt);
  void MainLoop();

  void LoadRobotPose(std::vector<std::string> pose_str);
  void AddLineFeature(std::vector<std::string> line_str);
};