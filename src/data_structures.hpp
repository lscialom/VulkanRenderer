#pragma once

#include <Eigen/Core>
#include <vulkan/vulkan.hpp>

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

  // Non-wrapped type for constexpr qualifier
  static constexpr VkVertexInputBindingDescription GetBindingDescription() {
    return {0, sizeof(LiteralVertex), VK_VERTEX_INPUT_RATE_VERTEX};
  }

  // Non-wrapped type for constexpr qualifier
  static constexpr auto GetAttributeDescription() {
    return std::array<VkVertexInputAttributeDescription, 2>{
        {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LiteralVertex, pos)},
         {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LiteralVertex, nor)}}};
  }
};

bool operator==(const LiteralVertex &v1, const LiteralVertex &v2) {
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

template <typename T> struct UniformObjectInfo {
  static constexpr const uint64_t Size = sizeof(T);
  static constexpr const vk::DescriptorType DescriptorType =
      vk::DescriptorType::eUniformBuffer;

  // Non-wrapped type for constexpr qualifier
  static constexpr const VkShaderStageFlags ShaderStage = 0;

  uint32_t binding;
  uint32_t arraySize = 1;
  vk::Sampler *immutableSamplers = nullptr;

  vk::DescriptorSetLayoutBinding make_descriptor_set_layout_binding() {
    return vk::DescriptorSetLayoutBinding(binding, DescriptorType, arraySize,
                                          vk::ShaderStageFlags(ShaderStage),
                                          immutableSamplers);
  }
};

#define DEFINE_UNIFORM_OBJECT(type, descType, shaderStageFlags)                \
  template <>                                                                  \
  constexpr const vk::DescriptorType UniformObjectInfo<type>::DescriptorType = \
      descType;                                                                \
                                                                               \
  template <>                                                                  \
  constexpr const VkShaderStageFlags UniformObjectInfo<type>::ShaderStage =    \
      (VkShaderStageFlags)shaderStageFlags;

// UNIFORMBUFFERINFO SPECIALIZATIONS

struct CameraUBO {
  Eigen::Matrix4f view;
  Eigen::Matrix4f proj;
  Eigen::Vector3f viewPos;
};

DEFINE_UNIFORM_OBJECT(CameraUBO, vk::DescriptorType::eUniformBuffer,
                      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT)

struct LightUBO {
  Eigen::Vector4f vector;

  Eigen::Vector3f color;
  float ambientFactor;

  float maxDist;
};

DEFINE_UNIFORM_OBJECT(LightUBO, vk::DescriptorType::eUniformBuffer,
                      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT)

#undef DEFINE_UNIFORM_OBJECT

//-----------------------------------------------------------------------------
// PUSH CONSTANTS
//-----------------------------------------------------------------------------

template <typename PushConstantStruct> struct PushConstantDescriptor {
  static constexpr uint32_t Size = sizeof(PushConstantStruct);
  static constexpr VkShaderStageFlags ShaderStageFlags = 0;
  uint32_t Offset = 0;

  vk::PushConstantRange make_push_constant_range() {
    return vk::PushConstantRange(vk::ShaderStageFlags(ShaderStageFlags), Offset,
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
  Eigen::Matrix4f model;
  Eigen::Vector4f color;
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

struct SSAOPassPushConstant
{
	float xExtent;
	float yExtent;
};

DEFINE_PUSH_CONSTANT(SSAOPassPushConstant,
	VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

#undef DEFINE_PUSH_CONSTANT
