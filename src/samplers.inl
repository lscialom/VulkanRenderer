//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

#define DEFINE_SAMPLER(name, ...)                                              \
  vk::Sampler name;                                                            \
  static constexpr VkSamplerCreateInfo name##Info = {                          \
      .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO, __VA_ARGS__};

#define CREATE_SAMPLER(sampler)                                                \
  g_device.createSampler(                                                      \
      reinterpret_cast<const vk::SamplerCreateInfo *>(&sampler##Info),         \
      g_allocationCallbacks, &sampler);

#define DESTROY_SAMPLER(sampler)                                               \
  g_device.destroySampler(sampler, g_allocationCallbacks);

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------

// clang-format off

DEFINE_SAMPLER(BaseSampler, 
		.magFilter = VK_FILTER_NEAREST,
		.minFilter = VK_FILTER_NEAREST,
		.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST,
		.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
		.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
		.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
		.mipLodBias = 0.f,
		.anisotropyEnable = false,
		.maxAnisotropy = 0,
		.compareEnable = false,
		.compareOp = VK_COMPARE_OP_ALWAYS,
		.minLod = 0,
		.maxLod = 0,
		.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
		.unnormalizedCoordinates = false
)

DEFINE_SAMPLER(RepeatSampler,
	.magFilter = VK_FILTER_NEAREST,
	.minFilter = VK_FILTER_NEAREST,
	.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST,
	.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT,
	.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT,
	.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT,
	.mipLodBias = 0.f,
	.anisotropyEnable = false,
	.maxAnisotropy = 0,
	.compareEnable = false,
	.compareOp = VK_COMPARE_OP_ALWAYS,
	.minLod = 0,
	.maxLod = 0,
	.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
	.unnormalizedCoordinates = false
)

// clang-format on

#undef DEFINE_SAMPLER

//-----------------------------------------------------------------------------
// INITIALIZER - DESTROYER
//-----------------------------------------------------------------------------

void InitSamplers() {
  CREATE_SAMPLER(BaseSampler);
  CREATE_SAMPLER(RepeatSampler);
}

void DestroySamplers() {
  DESTROY_SAMPLER(BaseSampler);
  DESTROY_SAMPLER(RepeatSampler);
}

#undef DESTROY_SAMPLER
#undef CREATE_SAMPLER
