#include <Eigen/Core>
#include <visualization/vis.h>

std::vector<Eigen::VectorXd> DrawEllipse(Eigen::Vector2d center,
                                         Eigen::Vector2d major_axis, Eigen::Vector2d minor_axis,
                                         size_t n_pts);