//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

#define DEFINE_BUFFER(name, ...)                                               \
  ::Renderer::Buffer name;                                                     \
  static const BufferInfo name##Info = {__VA_ARGS__};

#define DEFINE_TEXTURE2D(name, ...)                                            \
  ::Renderer::Image name;                                                      \
  static const Texture2DInfo name##Info = {__VA_ARGS__};

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------

// clang-format off

DEFINE_BUFFER(SSAOKernelBuffer,
	.size = SSAO_NUM_SAMPLES * sizeof(Eigen::Vector4f),
	.usage = vk::BufferUsageFlagBits::eTransferDst
				| vk::BufferUsageFlagBits::eUniformBuffer,
	.memProperties = vk::MemoryPropertyFlagBits::eDeviceLocal
)

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
#undef DEFINE_BUFFER

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
  SSAOKernelBuffer.allocate(SSAOKernelBufferInfo);

  SSAOKernelBuffer.write(ssaoKernelData.data(),
                         ssaoKernelData.size() *
                             sizeof(*ssaoKernelData.data()));
}

void DestroySSAOResources() {
  SSAONoiseTex.free();

  SSAOKernelBuffer.~Buffer();
}
