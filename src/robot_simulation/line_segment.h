#pragma once

#include <Eigen/Core>

class LineSegment {
public:
  Eigen::Vector2d p1, p2;

  LineSegment(Eigen::Vector2d p1, Eigen::Vector2d p2);

  bool CheckIntersection(Eigen::Vector2d p, Eigen::Vector2d v, double &d);
  double Distance(Eigen::Vector2d p);
};