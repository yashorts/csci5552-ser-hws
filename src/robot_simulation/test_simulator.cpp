#include <robot_simulation/simulator.h>
#include <visualization/gvis.h>

int main() {
  std::string scene_name = "simple_rectangle.scn";

  Simulator sim(scene_name);
  GVisualizer vis;

  size_t scan_fig_id = vis.AddFigure("Scan", true);
  vis.ShowFigures();
  // Wait a second to initialize everything in the simulator
  vis.PauseFigures(1);


  Eigen::Vector2d ctrl(0.3,0.5);
  sim.SendControl(ctrl);

  std::vector<Eigen::VectorXd> poses;
  for (size_t i = 0; i < 1000; ++i) {
    // Check the sensors to see if there's new information
    OdometryData odom = sim.GetOdometry();
    LaserScanData scan = sim.GetLaserScan();

    // ------------------
    //    Drawing Code
    // ------------------
    auto t_start = std::chrono::high_resolution_clock::now();
    // For visualization purposes only, you shouldn't use this in your localization
    poses.push_back(sim.GetTruePose());

    // Clear all old lines
    vis.ClearFigure(scan_fig_id);

    // Draw the building
    for (auto& line : sim.GetLines()) {
      vis.Plot(scan_fig_id, line, "blue");
    }

    // Draw the Laser scan:
    std::vector<Eigen::VectorXd> line;
    Eigen::Vector2d base = poses.back().head<2>() - 0.5 * sim.robot_radius * Eigen::Vector2d(std::cos(poses.back()[2]), std::sin(poses.back()[2]));
    line.push_back(base);
    for (size_t i = 0; i < sim.n_lasers; ++i) {
      if (scan.scan[i] == sim.laser_dist_overdist_val) {
        continue;
      }
      double perc = static_cast<double>(i) / (static_cast<double>(sim.n_lasers) - 1.0);
      double th = sim.min_th + (sim.max_th - sim.min_th) * perc;

      Eigen::VectorXd scan_pt_i = base + scan.scan[i] * Eigen::Vector2d(std::cos(th+poses.back()[2]), std::sin(th+poses.back()[2]));

      line.push_back(scan_pt_i);
      vis.Plot(scan_fig_id, line, "red");
      line.pop_back();
    }

    // Update the figure
    vis.DrawFigure(scan_fig_id);
    vis.SetFigureAxisLimits(scan_fig_id, Eigen::Vector2d(poses.back()[0]-10,poses.back()[0]+10), Eigen::Vector2d(poses.back()[1]-10,poses.back()[1]+10));

    auto t_now = std::chrono::high_resolution_clock::now();
    auto iter_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now - t_start).count();
    float iter_dur_ms = static_cast<float>(iter_dur) / 1e6;
    std::cout << "Drawing takes: " << iter_dur_ms << "ms" << std::endl;

    // Sleep for 10 milliseconds before running again
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sim.SendControl(Eigen::Vector2d::Zero());
  vis.PauseFigures(-1);
}