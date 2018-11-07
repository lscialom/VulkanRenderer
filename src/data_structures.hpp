#include <Eigen/Dense>
#include <vulkan/vulkan.hpp>

//-----------------------------------------------------------------------------
// VERTEX RELATED
//-----------------------------------------------------------------------------

struct Vertex {
  Eigen::Vector2f pos;
  Eigen::Vector3f color;

  static vk::VertexInputBindingDescription GetBindingDescription() {
    return vk::VertexInputBindingDescription(0, sizeof(Vertex),
                                             vk::VertexInputRate::eVertex);
  }

  static std::array<vk::VertexInputAttributeDescription, 2>
  GetAttributeDescription() {
    return std::array<vk::VertexInputAttributeDescription, 2>{
        {{0, 0, vk::Format::eR32G32Sfloat, offsetof(Vertex, pos)},
         {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(Vertex, color)}}};
  }
};

// TODO Temporary
static const std::vector<Vertex> g_vertices = {
    {{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
    {{0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}},
    {{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
    {{-0.5f, 0.5f}, {1.0f, 1.0f, 1.0f}}};

static const std::vector<VERTEX_INDICES_TYPE> g_indices = {0, 1, 2, 2, 3, 0};

//-----------------------------------------------------------------------------
// UBO RELATED
//-----------------------------------------------------------------------------

template <typename T> struct UniformBufferInfo {
  static constexpr const uint64_t size = sizeof(T);
  static constexpr const vk::DescriptorType descriptorType =
      vk::DescriptorType::eUniformBuffer;
  static const vk::ShaderStageFlags shaderStage;

  uint32_t binding;
  uint32_t arraySize = 1;
  vk::Sampler *immutableSamplers = nullptr;

  vk::DescriptorSetLayoutBinding make_descriptor_set_layout_binding() {
    return vk::DescriptorSetLayoutBinding(binding, descriptorType, arraySize,
                                          shaderStage, immutableSamplers);
  }
};

#define DEFINE_UBO(type, descType, shaderStageFlags)                           \
  template <>                                                                  \
  constexpr const vk::DescriptorType UniformBufferInfo<type>::descriptorType = \
      descType;                                                                \
                                                                               \
  template <>                                                                  \
  const vk::ShaderStageFlags UniformBufferInfo<type>::shaderStage =            \
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
