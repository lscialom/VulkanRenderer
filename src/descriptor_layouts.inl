// TODO Array of layouts for Passes

//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

#define DEFINE_LAYOUT(name, bindingCount, ...)                                 \
  static constexpr std::array<VkDescriptorSetLayoutBinding, bindingCount>      \
      name##LayoutInfo = {__VA_ARGS__};

#define DEFINE_LAYOUT_BINDING(...)                                             \
  { __VA_ARGS__ }

#define CREATE_DESCRIPTOR_SET_LAYOUT(layoutBindings, dstLayout)                \
  vk::DescriptorSetLayoutCreateInfo layoutBindings##layoutInfo(                \
      vk::DescriptorSetLayoutCreateFlags(),                                    \
      static_cast<uint32_t>(layoutBindings.size()),                            \
      reinterpret_cast<const vk::DescriptorSetLayoutBinding *>(                \
          layoutBindings.data()));                                             \
                                                                               \
  CHECK_VK_RESULT_FATAL(                                                       \
      g_device.createDescriptorSetLayout(&layoutBindings##layoutInfo,          \
                                         g_allocationCallbacks, &dstLayout),   \
      "Failed to create descriptor set layout.");

#define DESTROY_DESCRIPTOR_SET_LAYOUT(layout)                                  \
  g_device.destroyDescriptorSetLayout(layout, g_allocationCallbacks)

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------

// clang-format off

vk::DescriptorSetLayout UniqueTextureLayout;
DEFINE_LAYOUT(UniqueTexture, 1,
	{
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
		)
	})

vk::DescriptorSetLayout GBufferLayout;
DEFINE_LAYOUT(GBuffer, 1,
	{
		// G-Buffer attachments
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = G_BUFFER_SIZE,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
		)
	})

vk::DescriptorSetLayout SSAOLayout;
DEFINE_LAYOUT(SSAO, 2,
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
  CREATE_DESCRIPTOR_SET_LAYOUT(UniqueTextureLayoutInfo, UniqueTextureLayout);
  CREATE_DESCRIPTOR_SET_LAYOUT(GBufferLayoutInfo, GBufferLayout);
  CREATE_DESCRIPTOR_SET_LAYOUT(SSAOLayoutInfo, SSAOLayout);
}

void DestroyDescriptorLayouts() {
  DESTROY_DESCRIPTOR_SET_LAYOUT(UniqueTextureLayout);
  DESTROY_DESCRIPTOR_SET_LAYOUT(GBufferLayout);
  DESTROY_DESCRIPTOR_SET_LAYOUT(SSAOLayout);
}

#undef DESTROY_DESCRIPTOR_SET_LAYOUT
#undef CREATE_DESCRIPTOR_SET_LAYOUT

#define DESCRIPTOR_INFO(layout) layout##Info, layout
