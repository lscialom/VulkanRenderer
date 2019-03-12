::Renderer::Image SSAONoiseTex;
::Renderer::Buffer SSAOKernel;

void InitSSAOResources() {

  SSAONoiseTex.allocate(
      SSAO_NOISE_DIM, SSAO_NOISE_DIM, vk::Format::eR32G32B32A32Sfloat,
      vk::ImageTiling::eOptimal,
      vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
      vk::MemoryPropertyFlagBits::eDeviceLocal);

  SSAONoiseTex.transition_layout(vk::ImageLayout::eUndefined,
                                 vk::ImageLayout::eTransferDstOptimal);

  std::vector<Eigen::Vector4f> ssaoNoise =
      Maths::GenerateSSAONoise(SSAO_NOISE_DIM);

  ::Renderer::Buffer stagingBuffer;
  stagingBuffer.allocate(ssaoNoise.size() * sizeof(*ssaoNoise.data()),
                         vk::BufferUsageFlagBits::eTransferSrc,
                         vk::MemoryPropertyFlagBits::eHostVisible |
                             vk::MemoryPropertyFlagBits::eHostCoherent);

  stagingBuffer.write(ssaoNoise.data(),
                      ssaoNoise.size() * sizeof(*ssaoNoise.data()));

  SSAONoiseTex.write_from_buffer(stagingBuffer.get_handle(), SSAO_NOISE_DIM,
                                 SSAO_NOISE_DIM);

  SSAONoiseTex.transition_layout(vk::ImageLayout::eTransferDstOptimal,
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
