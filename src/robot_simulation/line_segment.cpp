#include <robot_simulation/line_segment.h>
#include <Eigen/Dense>

LineSegment::LineSegment(Eigen::Vector2d p1_, Eigen::Vector2d p2_) {
  p1 = p1_;
  p2 = p2_;
}

bool LineSegment::CheckIntersection(Eigen::Vector2d pt, Eigen::Vector2d v, double &d) {
  const Eigen::Vector2d& pt2  = p1;
  Eigen::Vector2d v2 = p2 - p1;

  v.normalize(); // Make sure v is a unit vector

  // If the two directions are nearly parallel, we'll just assume no intersection
  if (std::abs(v.dot(v2.normalized())) > 1.0 - 1e-5) {
    return false;
  }

  Eigen::Matrix2d A;
  A.col(0) = -v;
  A.col(1) = v2;

  Eigen::Vector2d t1t2 = A.inverse() * (pt - pt2);

  // Check if the collision is off the ends of the line segment, or behind the robot
  if (t1t2[1] < 0 || t1t2[1] > 1 || t1t2[0] <= 0) {
    return false;
  }

  d = t1t2[0];
  return true;
}

double LineSegment::Distance(Eigen::Vector2d p) {
  Eigen::Vector2d v = p2 - p1;
  double v_n = v.norm();
  v.normalize();
  Eigen::Vector2d v_p = p - p1;

  double proj_len = v_p.dot(v);
  if (proj_len < 0) { // Projects to before p1
    return v_p.norm();
  } else if (proj_len > v_n) { // Projects to after p2
    return (p - p2).norm();
  } else { // Somewhere in the middle
    return (v_p - proj_len * v).norm();
  }
}