#pragma once

#include <Eigen/Core>

#include "configuration_helper.hpp"
#include "global_context.hpp"

//-----------------------------------------------------------------------------
// VERTEX INPUT DESCRIPTION
//-----------------------------------------------------------------------------

// For compile-time primitives
template <uint8_t N> struct LiteralVector { float members[N]; };

template <uint8_t N>
bool operator==(const LiteralVector<N> &v1, const LiteralVector<N> &v2) {
  for (uint8_t i = 0; i < N; ++i)
    if (v1.members[i] != v2.members[i])
      return false;
  return true;
}

struct LiteralVertex {
  LiteralVector<3> pos;
  LiteralVector<3> nor;
  LiteralVector<2> texcoords;

  // Non-wrapped type for constexpr qualifier
  static constexpr VkVertexInputBindingDescription GetBindingDescription() {
    return {0, sizeof(LiteralVertex), VK_VERTEX_INPUT_RATE_VERTEX};
  }

  // Non-wrapped type for constexpr qualifier
  static constexpr auto GetAttributeDescription() {
    return std::array<VkVertexInputAttributeDescription, 3>{
        {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LiteralVertex, pos)},
         {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LiteralVertex, nor)},
         {2, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(LiteralVertex, texcoords)}}};
  }
};

static bool operator==(const LiteralVertex &v1, const LiteralVertex &v2) {
  return v1.pos == v2.pos && v1.nor == v2.nor;
}

// template <size_t N> struct Vertex {
//  Eigen::Matrix<float, N, 1> pos;
//
//  // Non-wrapped type for constexpr qualifier
//  static constexpr VkVertexInputBindingDescription GetBindingDescription() {
//    return {0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
//  }
//
//  static constexpr VkFormat GetFormatFromDimension() {
//    switch (N) {
//    case 2:
//      return VK_FORMAT_R32G32_SFLOAT;
//    case 3:
//      return VK_FORMAT_R32G32B32_SFLOAT;
//    case 4:
//      return VK_FORMAT_R32G32B32A32_SFLOAT;
//    }
//  }
//
//  // Non-wrapped type for constexpr qualifier
//  static constexpr auto GetAttributeDescription() {
//    return std::array<VkVertexInputAttributeDescription, 1>{
//        {{0, 0, GetFormatFromDimension(), offsetof(Vertex, pos)}}};
//  }
//
//  static_assert(N >= 2 && N <= 4);
//};

//-----------------------------------------------------------------------------
// PRIMITIVES
//-----------------------------------------------------------------------------

// TODO Replace with preloaded meshes
template <size_t vNum, size_t iNum> struct Primitive {
  static constexpr const std::array<LiteralVertex, vNum> Vertices;
  static constexpr const std::array<VERTEX_INDICES_TYPE, iNum> Indices;

  using VType = decltype(Vertices);
  using IType = decltype(Indices);
};

#define DECLARE_PRIMITIVE(PrimName, vNum, iNum)                                \
  typedef Primitive<vNum, iNum> PrimName;

#define SET_PRIMITIVE_VERTICES(PrimType, ...)                                  \
  template <> PrimType::VType PrimType::Vertices = __VA_ARGS__;

#define SET_PRIMITIVE_INDICES(PrimType, ...)                                   \
  template <> PrimType::IType PrimType::Indices = __VA_ARGS__;

// PLANE
DECLARE_PRIMITIVE(Plane, 4, 6)
SET_PRIMITIVE_VERTICES(
    Plane, {{{1.f, -1.f, 0}, {-1.f, -1.f, 0}, {-1.f, 1.f, 0}, {1.f, 1.f, 0}}})

SET_PRIMITIVE_INDICES(Plane, {0, 1, 2, 2, 3, 0})

// CUBE
DECLARE_PRIMITIVE(Cube, 8, 36)
SET_PRIMITIVE_VERTICES(Cube, {{{-1.0, -1.0, 1.0},
                               {1.0, -1.0, 1.0},
                               {1.0, 1.0, 1.0},
                               {-1.0, 1.0, 1.0},
                               {-1.0, -1.0, -1.0},
                               {1.0, -1.0, -1.0},
                               {1.0, 1.0, -1.0},
                               {-1.0, 1.0, -1.0}}})

SET_PRIMITIVE_INDICES(Cube,
                      {0, 1, 2, 2, 3, 0, 1, 5, 6, 6, 2, 1, 7, 6, 5, 5, 4, 7,
                       4, 0, 3, 3, 7, 4, 4, 5, 1, 1, 0, 4, 3, 2, 6, 6, 7, 3})

#undef DECLARE_PRIMITIVE
#undef SET_PRIMITIVE_VERTICES
#undef SET_PRIMITIVE_INDICES

//-----------------------------------------------------------------------------
// UNIFORM OBJECTS
//-----------------------------------------------------------------------------

struct CameraUBO {
  Eigen::Matrix<float, 4, 4, Eigen::DontAlign> view;
  Eigen::Matrix<float, 4, 4, Eigen::DontAlign> proj;
  Eigen::Matrix<float, 3, 1, Eigen::DontAlign> viewPos;
};

struct LightUBO {
  struct LightData {
    Eigen::Vector4f vector;

    Eigen::Vector3f color;
    float ambientFactor;

    float maxDist;
  };

  std::array<LightData, MAX_LIGHTS> lights;
};

//-----------------------------------------------------------------------------
// PUSH CONSTANTS
//-----------------------------------------------------------------------------

template <typename PushConstantStruct> struct PushConstantDescriptor {
  static constexpr uint32_t Size = sizeof(PushConstantStruct);
  static constexpr VkShaderStageFlags ShaderStageFlags = 0;
  uint32_t offset = 0;

  vk::PushConstantRange make_push_constant_range() {
    return vk::PushConstantRange(vk::ShaderStageFlags(ShaderStageFlags), offset,
                                 Size);
  }
};

#define DEFINE_PUSH_CONSTANT(PushConstantStructType, stageFlags)               \
  template <>                                                                  \
  constexpr VkShaderStageFlags                                                 \
      PushConstantDescriptor<PushConstantStructType>::ShaderStageFlags =       \
          (VkShaderStageFlags)stageFlags;                                      \
  static_assert(sizeof(PushConstantStructType) <= 128);

struct ModelInstancePushConstant {
  struct ModelInstanceData {
    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> model;
    Eigen::Matrix<float, 4, 1, Eigen::DontAlign> color;
  } modelInstanceData;

  Eigen::Matrix<float, 3, 1, Eigen::DontAlign> submeshColor;
};

DEFINE_PUSH_CONSTANT(ModelInstancePushConstant,
                     VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

struct ExtentPushConstant {
  float xExtent;
  float yExtent;
  float ratio = 6;
};

DEFINE_PUSH_CONSTANT(ExtentPushConstant,
                     VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

struct LightPassPushConstant {
  uint32_t numLights;
};

DEFINE_PUSH_CONSTANT(LightPassPushConstant,
                     VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

struct SSAOPassPushConstant {
  float xExtent;
  float yExtent;
};

DEFINE_PUSH_CONSTANT(SSAOPassPushConstant,
                     VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

#undef DEFINE_PUSH_CONSTANT
