#include <robot_simulation/simulator.h>
#include <visualization/opengl/glvis.h>

int main() {
  std::string scene_name = "vancouver.scn";

  Simulator sim(scene_name);
  GLVisualizer vis;
  // Wait a second to initialize everything in the simulator
  vis.PauseFigure(1);
  // Draw the building
  for (auto& line : sim.GetLines()) {
    vis.AddPermLine(line, Color::BLUE, 1.5);
  }

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
    // For visualization purposes only, you shouldn't use this in your localization
    poses.push_back(sim.GetTruePose());


    // Draw the Laser scan:
    // Clear all old temporary lines
    vis.ClearTempLines();
    std::vector<Eigen::VectorXd> line;
    Eigen::Vector2d base = poses.back().head<2>() - 0.5 * sim.robot_radius * Eigen::Vector2d(std::cos(poses.back()[2]), std::sin(poses.back()[2]));
    line.push_back(base);
    for (size_t i = 0; i < sim.n_lasers; ++i) {
      if (scan.scan.size() == 0 || scan.scan[i] == sim.laser_dist_overdist_val) {
        continue;
      }
      double perc = static_cast<double>(i) / (static_cast<double>(sim.n_lasers) - 1.0);
      double th = sim.min_th + (sim.max_th - sim.min_th) * perc;

      Eigen::VectorXd scan_pt_i = base + scan.scan[i] * Eigen::Vector2d(std::cos(th+poses.back()[2]), std::sin(th+poses.back()[2]));

      line.push_back(scan_pt_i);
      vis.AddTempLine(line, Color::RED, 0.25);
      line.pop_back();
    }

    // Update the figure
    vis.UpdateLines();
    vis.SetCenterAndHeight(poses.back().head<2>(), 10.0);

    // Sleep for 10 milliseconds before running again
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sim.SendControl(Eigen::Vector2d::Zero());
  vis.PauseFigure(-1);
}