template <typename T> struct UniformObjectLayout {
  static constexpr const uint64_t Size = sizeof(T);
  static constexpr const vk::DescriptorType DescriptorType =
      vk::DescriptorType::eUniformBuffer;

  // Non-wrapped type for constexpr qualifier
  static constexpr const VkShaderStageFlags ShaderStage = 0;

  static constexpr uint32_t Binding = 0;
  static constexpr uint32_t ArraySize = 1;

  vk::DescriptorSetLayout layout;

  void init() {

    vk::DescriptorSetLayoutBinding layoutBinding(
        Binding, DescriptorType, ArraySize, vk::ShaderStageFlags(ShaderStage),
        nullptr);

    vk::DescriptorSetLayoutCreateInfo layoutInfo(
        vk::DescriptorSetLayoutCreateFlags(), 1, &layoutBinding);

    CHECK_VK_RESULT_FATAL(g_device.createDescriptorSetLayout(
                              &layoutInfo, g_allocationCallbacks, &layout),
                          "Failed to create descriptor set layout.");
  }

  void destroy() {
    if (layout)
      g_device.destroyDescriptorSetLayout(layout, g_allocationCallbacks);

    layout = nullptr;
  }
};

//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

#define DEFINE_UBO(type, descType, shaderStageFlags, binding, arraySize)       \
  UniformObjectLayout<type> type##Layout;                                      \
                                                                               \
  template <>                                                                  \
  constexpr const vk::DescriptorType                                           \
      UniformObjectLayout<type>::DescriptorType = descType;                    \
                                                                               \
  template <>                                                                  \
  constexpr const VkShaderStageFlags UniformObjectLayout<type>::ShaderStage =  \
      (VkShaderStageFlags)shaderStageFlags;                                    \
                                                                               \
  template <>                                                                  \
  constexpr const uint32_t UniformObjectLayout<type>::Binding = binding;       \
                                                                               \
  template <>                                                                  \
  constexpr const uint32_t UniformObjectLayout<type>::ArraySize = arraySize;

#define CREATE_UBO(dataType) dataType##Layout.init();

#define DESTROY_UBO(dataType) dataType##Layout.destroy();

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------

DEFINE_UBO(CameraUBO, vk::DescriptorType::eUniformBuffer,
           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, 1)

DEFINE_UBO(LightUBO, vk::DescriptorType::eUniformBuffer,
           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
           MAX_LIGHTS)

#undef DEFINE_UBO

//-----------------------------------------------------------------------------
// INITIALIZER - DESTROYER
//-----------------------------------------------------------------------------

void InitUniformBufferObjects() {
  CREATE_UBO(CameraUBO);
  CREATE_UBO(LightUBO);
}

void DestroyUniformBufferObjects() {
  DESTROY_UBO(CameraUBO);
  DESTROY_UBO(LightUBO);
}

#undef CREATE_UBO
#undef DESTROY_UBO
