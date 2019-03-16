//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

#define DEFINE_TEXTURE2D(name, ...)                                            \
  ::Renderer::Image name;                                                      \
  static const Texture2DInfo name##Info = {__VA_ARGS__};

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------

::Renderer::Buffer SSAOKernel;

// clang-format off

DEFINE_TEXTURE2D(SSAONoiseTex, 
	.texWidth = SSAO_NOISE_DIM,
	.texHeight = SSAO_NOISE_DIM,
	.format = vk::Format::eR32G32B32A32Sfloat,
	.tiling = vk::ImageTiling::eOptimal,
	.usage = vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
	.memProperties = vk::MemoryPropertyFlagBits::eDeviceLocal,
	.aspectFlags = vk::ImageAspectFlagBits::eColor
)

// clang-format on

#undef DEFINE_TEXTURE2D

//-----------------------------------------------------------------------------
// INITIALIZER - DESTROYER
//-----------------------------------------------------------------------------

void InitSSAOResources() {

  SSAONoiseTex.allocate(SSAONoiseTexInfo);

  std::vector<Eigen::Vector4f> ssaoNoise =
      Maths::GenerateSSAONoise(SSAO_NOISE_DIM);

  SSAONoiseTex.write_from_raw_data(ssaoNoise.data(),
                                   vk::ImageLayout::eUndefined,
                                   vk::ImageLayout::eShaderReadOnlyOptimal);

  std::vector<Eigen::Vector4f> ssaoKernelData =
      Maths::GenerateSSAOKernel(SSAO_NUM_SAMPLES);

  // TODO DeviceLocal
  SSAOKernel.allocate(ssaoKernelData.size() * sizeof(*ssaoKernelData.data()),
                      vk::BufferUsageFlagBits::eUniformBuffer,
                      vk::MemoryPropertyFlagBits::eHostVisible |
                          vk::MemoryPropertyFlagBits::eHostCoherent);

  SSAOKernel.write(ssaoKernelData.data(),
                   ssaoKernelData.size() * sizeof(*ssaoKernelData.data()));
}

void DestroySSAOResources() {
  SSAONoiseTex.free();

  SSAOKernel.~Buffer();
}
