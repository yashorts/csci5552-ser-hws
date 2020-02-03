#include <homework/hw0/hw0.h>

std::vector<Eigen::VectorXd> DrawEllipse(Eigen::Vector2d center,
                                         Eigen::Vector2d major_axis, Eigen::Vector2d minor_axis,
                                         size_t n_pts) {
  std::vector<Eigen::VectorXd> points(n_pts);

  // Compute parametric ellipse parameters:
  double ellipse_orientation = std::atan2(major_axis[1], major_axis[0]);
  double scale_a = major_axis.norm();
  double scale_b = minor_axis.norm();
  Eigen::Matrix2d Rot;
  Rot << std::cos(ellipse_orientation), -std::sin(ellipse_orientation), std::sin(ellipse_orientation), std::cos(ellipse_orientation);

  for (size_t i = 0; i < n_pts; ++i) {
    // n_pts-1 so that the first and last points are the same (completing the
    // circle)
    double theta = 6.28318530718 * i / (n_pts - 1);

    // Scale by major / minor axis, then rotate and offset
    points[i] = Eigen::Vector2d(scale_a * std::cos(theta), scale_b * std::sin(theta));
    points[i] = Rot * points[i] + center;
  }

  return points;
}