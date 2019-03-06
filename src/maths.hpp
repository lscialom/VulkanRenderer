#pragma once

#include <Eigen/Dense>

#include <random>
#include <array>

namespace Maths {
static const uint32_t DitheringPatternDim = 8;
static std::array<float, DitheringPatternDim * DitheringPatternDim> DitheringPattern = {
    0,  32, 8,  40, 2,  34, 10, 42, /* 8x8 Bayer ordered dithering  */
    48, 16, 56, 24, 50, 18, 58, 26, /* pattern.  Each input pixel   */
    12, 44, 4,  36, 14, 46, 6,  38, /* is scaled to the 0..63 range */
    60, 28, 52, 20, 62, 30, 54, 22, /* before looking in this table */
    3,  35, 11, 43, 1,  33, 9,  41, /* to determine the action.     */
    51, 19, 59, 27, 49, 17, 57, 25, 15, 47, 7,  39,
    13, 45, 5,  37, 63, 31, 55, 23, 61, 29, 53, 21};

template <typename VecType>
static Eigen::Vector3f EigenizeVec3(const VecType &vec) {

  static_assert(sizeof(Eigen::Vector3f) == sizeof(VecType));

  Eigen::Vector3f res{};
  memcpy((void *)&res, (void *)&vec, sizeof(VecType));

  return res;
}

static float DegToRad(float angle) { return angle * EIGEN_PI / 180.f; }
static float RadToDeg(float angle) { return angle * 180.f / EIGEN_PI; }

static float Lerp(float a, float b, float alpha) { return a + alpha * (b - a); }

static std::vector<Eigen::Vector4f> GenerateSSAOKernel(uint32_t nbSamples) {

  std::random_device RandomDevice; // obtain a random number from hardware

  std::uniform_real_distribution<float> randomFloats(
      0.0, 1.0); // random floats between 0.0 - 1.0
  std::default_random_engine generator(RandomDevice());

  std::vector<Eigen::Vector4f> ssaoKernel;
  for (uint32_t i = 0; i < nbSamples; ++i) {
    Eigen::Vector4f sample(randomFloats(generator) * 2.0 - 1.0,
                           randomFloats(generator) * 2.0 - 1.0,
                           randomFloats(generator), 0);

    sample.normalize();
    sample *= randomFloats(generator);

    float scale = (float)i / (float)nbSamples;
    scale = Lerp(0.1f, 1.0f, scale * scale);

    sample *= scale;

    sample.w() = 1;
    ssaoKernel.push_back(sample);
  }

  return ssaoKernel;
}

static std::vector<Eigen::Vector4f> GenerateSSAONoise(uint32_t dimension) {

  std::random_device RandomDevice; // obtain a random number from hardware

  std::uniform_real_distribution<float> randomFloats(
      0.0, 1.0); // random floats between 0.0 - 1.0
  std::default_random_engine generator(RandomDevice());

  std::vector<Eigen::Vector4f> ssaoNoise;
  for (uint32_t i = 0; i < dimension * dimension; i++) {
    Eigen::Vector4f noise(randomFloats(generator) * 2.0 - 1.0,
                          randomFloats(generator) * 2.0 - 1.0, 0.0f, 0);

    ssaoNoise.push_back(noise);
  }

  return ssaoNoise;
}

static Eigen::Matrix4f Perspective(double fovy, double aspect, double zNear,
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

static Eigen::Matrix4f LookAt(Eigen::Vector3f const &eye,
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
