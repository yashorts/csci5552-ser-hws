#include <Eigen/Core>
#include <cmath>
#include <homework/hw0/hw0.h>

int main() {
  Visualizer vis;
  size_t fig_id = vis.AddFigure("Ellipse", true);
  vis.ShowFigures();

  Eigen::Vector2d center = Eigen::Vector2d(1.1, 2.2);
  Eigen::Vector2d major_axis = Eigen::Vector2d(std::cos(1.1), std::sin(1.1));
  Eigen::Vector2d minor_axis =
      Eigen::Vector2d(-0.2 * std::sin(1.1), 0.2 * std::cos(1.1));

  std::vector<Eigen::VectorXd> pts;
  pts.push_back(center);
  pts.push_back(major_axis + center);
  vis.Plot(fig_id, pts, "b-x");

  pts.pop_back();
  pts.push_back(minor_axis + center);
  vis.Plot(fig_id, pts, "b-x");

  std::vector<Eigen::VectorXd> ellipse_points = DrawEllipse(center, major_axis, minor_axis, 100);
  vis.Plot(fig_id, ellipse_points, "r-");
  vis.PauseFigures(-1);

  return 0;
}