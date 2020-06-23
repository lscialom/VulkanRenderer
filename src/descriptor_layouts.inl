//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

#define DEFINE_LAYOUT(name, bindingCount, ...)                                 \
  ::Renderer::DescriptorLayout name;                                           \
  static constexpr std::array<VkDescriptorSetLayoutBinding, bindingCount>      \
      name##BindingsInfo = {__VA_ARGS__};

#define DEFINE_LAYOUT_BINDING(...)                                             \
  { __VA_ARGS__ }

#define CREATE_DESCRIPTOR_SET_LAYOUT(dstLayout)                                \
  vk::DescriptorSetLayoutCreateInfo dstLayout##layoutInfo(                     \
      vk::DescriptorSetLayoutCreateFlags(),                                    \
      static_cast<uint32_t>(dstLayout##BindingsInfo.size()),                   \
      reinterpret_cast<const vk::DescriptorSetLayoutBinding *>(                \
          dstLayout##BindingsInfo.data()));                                    \
                                                                               \
  CHECK_VK_RESULT_FATAL(                                                       \
      g_device.createDescriptorSetLayout(                                      \
          &dstLayout##layoutInfo, g_allocationCallbacks, &dstLayout.handle),   \
      "Failed to create descriptor set layout.");                              \
                                                                               \
  dstLayout.bindings.resize(dstLayout##BindingsInfo.size());                   \
  memcpy(dstLayout.bindings.data(), dstLayout##BindingsInfo.data(),            \
         dstLayout##BindingsInfo.size() *                                      \
             sizeof(VkDescriptorSetLayoutBinding));

#define DESTROY_DESCRIPTOR_SET_LAYOUT(layout)                                  \
  g_device.destroyDescriptorSetLayout(layout.handle, g_allocationCallbacks);   \
  layout.handle = nullptr;                                                     \
  layout.bindings.clear();

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------

// clang-format off

DEFINE_LAYOUT(UniqueTextureLayout, 1,
	{
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
		)
	})

DEFINE_LAYOUT(MeshLayout, 1,
	{
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
		)
	})

DEFINE_LAYOUT(GBufferLayout, 1,
	{
		// G-Buffer attachments
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = G_BUFFER_SIZE,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
		)
	})

// TODO Move to ssao_resources.inl (or along passes when done)
DEFINE_LAYOUT(SSAOLayout, 2,
	{
		// SSAO Noise Rotations
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT
		),

		// SSAO Sample Kernel
		DEFINE_LAYOUT_BINDING(
			.binding = 1,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT
		),
	});

// clang-format on

#undef DEFINE_LAYOUT
#undef DEFINE_LAYOUT_BINDING

//-----------------------------------------------------------------------------
// INITIALIZER - DESTROYER
//-----------------------------------------------------------------------------

void InitDescriptorLayouts() {
  CREATE_DESCRIPTOR_SET_LAYOUT(UniqueTextureLayout);
  CREATE_DESCRIPTOR_SET_LAYOUT(MeshLayout);
  CREATE_DESCRIPTOR_SET_LAYOUT(GBufferLayout);
  CREATE_DESCRIPTOR_SET_LAYOUT(SSAOLayout);
}

void DestroyDescriptorLayouts() {
  DESTROY_DESCRIPTOR_SET_LAYOUT(UniqueTextureLayout);
  DESTROY_DESCRIPTOR_SET_LAYOUT(MeshLayout);
  DESTROY_DESCRIPTOR_SET_LAYOUT(GBufferLayout);
  DESTROY_DESCRIPTOR_SET_LAYOUT(SSAOLayout);
}

#undef DESTROY_DESCRIPTOR_SET_LAYOUT
#undef CREATE_DESCRIPTOR_SET_LAYOUT
