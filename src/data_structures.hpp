#include <Eigen/Core>
#include <vulkan/vulkan.hpp>

//-----------------------------------------------------------------------------
// VERTEX INPUT DESCRIPTION
//-----------------------------------------------------------------------------

// For compile-time primitives
template <uint8_t N> struct LiteralVector { float members[N]; };

struct LiteralVertex {
  LiteralVector<3> pos;
};

template <size_t N> struct Vertex {
  Eigen::Matrix<float, N, 1> pos;

  // Not using wrapped types here since it would prevent the function to be
  // compile time
  static constexpr VkVertexInputBindingDescription GetBindingDescription() {
    return {0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
  }

  static constexpr VkFormat GetFormatFromDimension() {
    switch (N) {
    case 2:
      return VK_FORMAT_R32G32_SFLOAT;
    case 3:
      return VK_FORMAT_R32G32B32_SFLOAT;
    case 4:
      return VK_FORMAT_R32G32B32A32_SFLOAT;
    }
  }

  // Not using wrapped types here since it would prevent the function to be
  // compile time
  static constexpr auto GetAttributeDescription() {
    return std::array<VkVertexInputAttributeDescription, 1>{
        {{0, 0, GetFormatFromDimension(), offsetof(Vertex, pos)}}};
  }

  static_assert(N != 0);
};

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

// SQUARE
DECLARE_PRIMITIVE(Square, 4, 6)
SET_PRIMITIVE_VERTICES(
    Square,
    {{{-0.5f, -0.5f, 0}, {0.5f, -0.5f, 0}, {0.5f, 0.5f, 0}, {-0.5f, 0.5f, 0}}})

SET_PRIMITIVE_INDICES(Square, {0, 1, 2, 2, 3, 0})

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
#undef DEFINE_PRIMITIVE_VERTICES
#undef DEFINE_PRIMITIVE_INDICES

//-----------------------------------------------------------------------------
// UBO RELATED
//-----------------------------------------------------------------------------

template <typename T> struct UniformBufferInfo {
  static constexpr const uint64_t Size = sizeof(T);
  static constexpr const vk::DescriptorType DescriptorType =
      vk::DescriptorType::eUniformBuffer;
  static const vk::ShaderStageFlags ShaderStage;

  uint32_t binding;
  uint32_t arraySize = 1;
  vk::Sampler *immutableSamplers = nullptr;

  vk::DescriptorSetLayoutBinding make_descriptor_set_layout_binding() {
    return vk::DescriptorSetLayoutBinding(binding, DescriptorType, arraySize,
                                          ShaderStage, immutableSamplers);
  }
};

#define DEFINE_UBO(type, descType, shaderStageFlags)                           \
  template <>                                                                  \
  constexpr const vk::DescriptorType UniformBufferInfo<type>::DescriptorType = \
      descType;                                                                \
                                                                               \
  template <>                                                                  \
  const vk::ShaderStageFlags UniformBufferInfo<type>::ShaderStage =            \
      shaderStageFlags;

// UNIFORMBUFFERINFO SPECIALIZATIONS

// UniformModelMat
// struct UniformModelMat {
//  Eigen::Matrix4f model;
//};
//
// DEFINE_UBO(UniformModelMat, vk::DescriptorType::eUniformBufferDynamic,
//           vk::ShaderStageFlagBits::eVertex)

#undef DEFINE_UBO
