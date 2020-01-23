#include <homework/hw0/hw0.h>

std::vector<Eigen::VectorXd> DrawEllipse(Eigen::Vector2d center,
                                         Eigen::Vector2d major_axis, Eigen::Vector2d minor_axis,
                                         size_t n_pts) {
  std::vector<Eigen::VectorXd> points(n_pts);

  for (size_t i = 0; i < n_pts; ++i) {
    // n_pts-1 so that the first and last points are the same (completing the
    // circle)
    double theta = 6.28318530718 * i / (n_pts - 1);

    // HW0: Modify this to account for the center, major axis, and minor axis
    points[i] = Eigen::Vector2d(std::cos(theta), std::sin(theta));
  }

  return points;
}