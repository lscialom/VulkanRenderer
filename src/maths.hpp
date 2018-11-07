#pragma once

#include <Eigen/Dense>

namespace Maths {
float DegToRad(float angle) { return angle * EIGEN_PI / 180.f; }
float RadToDeg(float angle) { return angle * 180.f / EIGEN_PI; }

Eigen::Matrix4f Perspective(double fovy, double aspect, double zNear,
                            double zFar) {
  using namespace Eigen;

  assert(aspect > 0);
  assert(zFar > zNear);

  double radf = DegToRad(fovy);
  double tanHalfFovy = tan(radf / 2.0);

  Matrix4f res = Matrix4f::Zero();
  res(0, 0) = 1.0 / (aspect * tanHalfFovy);
  res(1, 1) = 1.0 / (tanHalfFovy);
  res(2, 2) = -(zFar + zNear) / (zFar - zNear);
  res(3, 2) = -1.0;
  res(2, 3) = -(2.0 * zFar * zNear) / (zFar - zNear);

  return res;
}

Eigen::Matrix4f LookAt(Eigen::Vector3f const &eye,
                       Eigen::Vector3f const &center,
                       Eigen::Vector3f const &up) {
  using namespace Eigen;

  Vector3f f = (center - eye).normalized();
  Vector3f u = up.normalized();
  Vector3f s = f.cross(u).normalized();
  u = s.cross(f);

  Matrix4f res;
  res << s.x(), s.y(), s.z(), -s.dot(eye), u.x(), u.y(), u.z(), -u.dot(eye),
      -f.x(), -f.y(), -f.z(), f.dot(eye), 0, 0, 0, 1;

  return res;
}
} // namespace Maths
