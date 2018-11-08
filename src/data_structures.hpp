#include <Eigen/Core>
#include <vulkan/vulkan.hpp>

//-----------------------------------------------------------------------------
// VERTEX INPUT DESCRIPTION
//-----------------------------------------------------------------------------

struct Vertex {
  Eigen::Vector2f pos;
  Eigen::Vector3f color;

  // Not using wrapped types here since it would prevent the function to be
  // compile time
  static constexpr VkVertexInputBindingDescription GetBindingDescription() {
    return {0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
  }

  // Not using wrapped types here since it would prevent the function to be
  // compile time
  static constexpr std::array<VkVertexInputAttributeDescription, 2>
  GetAttributeDescription() {
    return std::array<VkVertexInputAttributeDescription, 2>{
        {{0, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, pos)},
         {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)}}};
  }
};

// TODO Temporary
static const std::vector<Vertex> g_vertices = {
    {{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
    {{0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}},
    {{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
    {{-0.5f, 0.5f}, {1.0f, 1.0f, 1.0f}}};

static constexpr const std::vector<VERTEX_INDICES_TYPE> g_indices = {0, 1, 2,
                                                                     2, 3, 0};

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

// UniformMVP
struct UniformMVP {
  Eigen::Matrix4f model;
  Eigen::Matrix4f view;
  Eigen::Matrix4f proj;
};

DEFINE_UBO(UniformMVP, vk::DescriptorType::eUniformBufferDynamic,
           vk::ShaderStageFlagBits::eVertex)

#undef DEFINE_UBO
