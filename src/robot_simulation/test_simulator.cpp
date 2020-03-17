#include <robot_simulation/simulator.h>
#include <visualization/vis.h>

int main() {
  std::string scene_name = "simple_rectangle.scn";

  Simulator sim(scene_name);
  Visualizer vis;

  size_t scan_fig_id = vis.AddFigure("Scan", true);

  for (auto& line : sim.GetLines()) {
    vis.Plot(scan_fig_id, line, "b-");
  }

  Eigen::Vector2d ctrl(0.3,0.5);
  sim.SendControl(ctrl);

  std::vector<Eigen::VectorXd> poses;
  for (size_t i = 0; i < 1000; ++i) {
    poses.push_back(sim.GetTruePose());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sim.SendControl(Eigen::Vector2d::Zero());

  for (size_t i = 0; i < 100; ++i) {
    poses.push_back(sim.GetTruePose());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  vis.Plot(scan_fig_id, poses, "g-x");


  LaserScanData scan = sim.GetLaserScan();

  std::vector<Eigen::VectorXd> line;
  line.push_back(Eigen::Vector2d::Zero());
  for (size_t i = 0; i < sim.n_lasers; ++i) {
    continue;
    if (scan.scan[i] == sim.laser_dist_overdist_val) {
      continue;
    }
    double perc = static_cast<double>(i) / (static_cast<double>(sim.n_lasers) - 1.0);
    double th = sim.min_th + (sim.max_th - sim.min_th) * perc;

    Eigen::VectorXd scan_pt_i = scan.scan[i] * Eigen::Vector2d(std::cos(th), std::sin(th));

    line.push_back(scan_pt_i);

    vis.Plot(scan_fig_id, line, "r-");

    line.pop_back();
  }
  vis.ShowFigures();
  vis.PauseFigures(-1);
}