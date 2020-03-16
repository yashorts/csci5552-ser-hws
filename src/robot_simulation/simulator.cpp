#include <robot_simulation/simulator.h>

#include <fstream>
#include <sstream>
#include <iostream>

std::vector<std::string> GetParts(std::string s, char delim) {
  std::stringstream ss(s);
  std::vector<std::string> parts;
  for (std::string part; std::getline(ss, part, delim); ) {
    parts.push_back(std::move(part));
  }
  return parts;
}

std::vector<std::vector<std::string>> LoadFileByToken(std::string file_name, int n_skip, char delim) {
  std::ifstream datafile(file_name);
  std::vector<std::vector<std::string>> data_vec;
  int n_lines_read = 0;
  for (std::string line; std::getline(datafile, line); ) {
    n_lines_read++;
    if (n_lines_read <= n_skip) {
      continue;
    }
    std::vector<std::string> parts = GetParts(line, delim);
    data_vec.push_back(parts);
  }
  datafile.close();
  return data_vec;
}

void Simulator::LoadRobotPose(std::vector<std::string> pose_str) {
  // Make sure the pose is of size 4 (x,y,th,radius)
  assert(pose_str.size() == 4);

  true_pose[0] = std::atof(pose_str[0].c_str());
  true_pose[1] = std::atof(pose_str[1].c_str());
  true_pose[2] = std::atof(pose_str[2].c_str());
  robot_radius = std::atof(pose_str[3].c_str());
}

void Simulator::AddLineFeature(std::vector<std::string> line_str) {
  // Make sure the line information is of size 4 (x1,y1,x2,y2)
  assert(line_str.size() == 4);

  Eigen::Vector2d p1(std::atof(line_str[0].c_str()), std::atof(line_str[1].c_str()));
  Eigen::Vector2d p2(std::atof(line_str[2].c_str()), std::atof(line_str[3].c_str()));

  line_features.emplace_back(p1,p2);
}

Simulator::Simulator(std::string map_file) {
  std::vector<std::vector<std::string>> file_contents = LoadFileByToken(std::string(SCENE_PATH) + map_file, 0, ' ');

  // Make sure we've loaded a non-empty map file.  Note that the first line in the file is the robot position
  assert(file_contents.size() > 0);

  LoadRobotPose(file_contents[0]);

  // Every Subsequent line in the file is a line segment:
  for (size_t i = 1; i < file_contents.size(); ++i) {
    AddLineFeature(file_contents[i]);
  }

  // Initialize the random generators:
  std::random_device rd;
  mt = std::mt19937(rd());
  double laser_ang_res = (max_th - min_th) / static_cast<double>(n_lasers);
  laser_th_err = std::uniform_real_distribution<double>(-th_err_perc * laser_ang_res, th_err_perc * laser_ang_res);
  laser_d_err = std::uniform_real_distribution<double>(-d_err, d_err);
  odom_v_err = std::uniform_real_distribution<double>(-v_err, v_err);
  odom_w_err = std::uniform_real_distribution<double>(-w_err, w_err);

  // Fork off the main simulation loop
  main_loop = std::thread(&Simulator::MainLoop, this);
}

Simulator::~Simulator() {
  running = false;
  if (main_loop.joinable()) {
    main_loop.join();
  }
}

std::vector<std::vector<Eigen::VectorXd>> Simulator::GetLines() {
  std::vector<std::vector<Eigen::VectorXd>> lines(line_features.size());
  for (size_t i = 0; i < line_features.size(); ++i) {
    lines[i].push_back(line_features[i].p1);
    lines[i].push_back(line_features[i].p2);
  }
  return lines;
}

Eigen::Vector3d Simulator::GetTruePose() {
  return true_pose;
}

void Simulator::MainLoop() {
  const auto start_time = std::chrono::high_resolution_clock::now();
  const auto loop_duration = std::chrono::milliseconds(10);
  double loop_dt = 1e-3 * loop_duration.count();

  int iter = 0;
  
  while (running) {
    if (iter%control_freq == 0) {
      // Do a control update
      UpdateCurrentControl(loop_dt);
      odometry_lock.lock();
      control_lock.lock();
      current_odom_data.odom = current_control;
      control_lock.unlock();
      current_odom_data.odom_time = std::chrono::high_resolution_clock::now();
      odometry_lock.unlock();
    }
    if (iter%laser_scan_freq == 0) {
      // Update the laser scan
      std::vector<double> tmp_scan = ComputeLaserScan();
      laser_scan_lock.lock();
      current_laser_scan.scan = tmp_scan;
      current_laser_scan.scan_time = std::chrono::high_resolution_clock::now();
      laser_scan_lock.unlock();
    }

    iter++;
    std::this_thread::sleep_until(start_time + iter * loop_duration);
  }
}

Eigen::Vector3d Simulator::ContDynamics(const Eigen::Vector3d& x_t, const Eigen::Vector2d& u_t) {
  Eigen::Vector3d x_dot_t;
  x_dot_t[0] = u_t[0] * std::cos(x_t[2]);
  x_dot_t[1] = u_t[0] * std::sin(x_t[2]);
  x_dot_t[2] = u_t[1];
  return x_dot_t;
}

void Simulator::UpdateCurrentControl(double dt) {
  Eigen::Vector2d tmp_control;
  control_lock.lock();
  // Only allow so much acceleration per timestep
  Eigen::Vector2d control_diff = goal_control - current_control;
  if (std::abs(control_diff[0]) > max_v_accel * dt) {
    control_diff[0] = control_diff[0] / std::abs(control_diff[0]) * max_v_accel * dt;
  }
  if (std::abs(control_diff[1]) > max_w_accel * dt) {
    control_diff[1] = control_diff[1] / std::abs(control_diff[1]) * max_w_accel * dt;
  }
  current_control += control_diff;
  tmp_control = current_control;
  control_lock.unlock();

  // Only apply noise if we're trying to move
  if (tmp_control.squaredNorm() != 0.0) {
    tmp_control[0] *= (1.0 + odom_v_err(mt));
    tmp_control[1] *= (1.0 + odom_w_err(mt));
  }

  // Run the dynamics via RK4
  Eigen::Vector3d k1, k2, k3, k4;
  Eigen::Vector3d x2, x3, x4;
  k1 = ContDynamics(true_pose, tmp_control);
  x2 = true_pose + 0.5f * dt * k1;
  k2 = ContDynamics(x2, tmp_control);
  x3 = true_pose + 0.5f * dt * k2;
  k3 = ContDynamics(x3, tmp_control);
  x4 = true_pose + dt * k3;
  k4 = ContDynamics(x4, tmp_control);

  true_pose = true_pose + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);

  for (auto& line : line_features) {
    if (line.Distance(true_pose.head<2>()) < robot_radius) {
      std::cerr << "Robot: \"Oh No! I crashed!!!!\"" << std::endl;
      running = false;
    }
  }
}

std::vector<double> Simulator::ComputeLaserScan() {
  std::vector<double> scan(n_lasers, laser_dist_overdist_val);
  // Move the center of the scanner back from the center of the robot
  Eigen::Vector2d scanner_center = true_pose.head<2>() - 0.5 * robot_radius * Eigen::Vector2d(std::cos(true_pose[2]), std::sin(true_pose[2]));
  for (size_t i = 0; i < n_lasers; ++i) {
    double perc = static_cast<double>(i) / (static_cast<double>(n_lasers) - 1.0);
    double th = min_th + (max_th - min_th) * perc + true_pose[2] + laser_th_err(mt);

    Eigen::Vector2d v(std::cos(th), std::sin(th));
    
    for (auto& line : line_features) {
      double seg_dist;
      if (line.CheckIntersection(scanner_center, v, seg_dist) && seg_dist < max_laser_dist) {
        scan[i] = std::min(scan[i], seg_dist);
      }
    }

    if (scan[i] < laser_dist_overdist_val) {
      scan[i] += laser_d_err(mt);
    }
  }

  return scan;
}

LaserScanData Simulator::GetLaserScan() {
  LaserScanData ret;
  laser_scan_lock.lock();
  ret = current_laser_scan;
  laser_scan_lock.unlock();
  return ret;
}
OdometryData Simulator::GetOdometry() {
  OdometryData ret;
  control_lock.lock();
  ret = current_odom_data;
  control_lock.unlock();
  return ret;
}
void Simulator::SendControl(Eigen::Vector2d ctrl) {
  control_lock.lock();
  goal_control = ctrl;
  control_lock.unlock();
}
bool Simulator::IsAlive() {
  return running;
}