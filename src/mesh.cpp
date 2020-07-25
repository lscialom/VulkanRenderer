#include "mesh.hpp"

#include "resource_manager.hpp"

#define DEFINE_SAMPLER(name, ...)                                              \
  VkSamplerCreateInfo name##Info = {                                           \
      .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO, __VA_ARGS__};

#define CREATE_SAMPLER(sampler)                                                \
  g_device.createSampler(                                                      \
      reinterpret_cast<const vk::SamplerCreateInfo *>(&sampler##Info),         \
      g_allocationCallbacks, &sampler);

namespace Renderer {
Mesh::Submesh &Mesh::Submesh::operator=(Submesh &&other) {
  indexCount = other.indexCount;
  indexOffset = other.indexOffset;

  diffuseColor = other.diffuseColor;

  diffuseSampler = std::move(other.diffuseSampler);
  other.diffuseSampler = nullptr;

  normalSampler = std::move(other.normalSampler);
  other.normalSampler = nullptr;

  descriptorSet = std::move(other.descriptorSet);

  return *this;
}

void Mesh::Submesh::init_descriptor(const std::string &diffuse,
                                    const std::string &normal) {

  DescriptorSetInfo descriptorSetInfo;
  descriptorSetInfo.layout = CommonResources::MeshLayout;

  descriptorSetInfo.images = {ResourceManager::GetTexture(diffuse)->get_image(),
                              ResourceManager::GetTexture(normal)->get_image()};

  // clang-format off

		// TODO Optimize sampler count
		DEFINE_SAMPLER(diffuseSampler,
			.magFilter = VK_FILTER_LINEAR,
			.minFilter = VK_FILTER_LINEAR,
			.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR,
			.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT,
			.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT,
			.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT,
			.mipLodBias = 0.f,
			.anisotropyEnable = true,
			.maxAnisotropy = 16,
			.compareEnable = false,
			.compareOp = VK_COMPARE_OP_ALWAYS,
			.minLod = 0,
			.maxLod = static_cast<float>(
				ResourceManager::GetTexture(diffuse)->get_mipLevels()),
			.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
			.unnormalizedCoordinates = false)

			DEFINE_SAMPLER(normalSampler,
				.magFilter = VK_FILTER_LINEAR,
				.minFilter = VK_FILTER_LINEAR,
				.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR,
				.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT,
				.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT,
				.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT,
				.mipLodBias = 0.f,
				.anisotropyEnable = true,
				.maxAnisotropy = 16,
				.compareEnable = false,
				.compareOp = VK_COMPARE_OP_ALWAYS,
				.minLod = 0,
				.maxLod = static_cast<float>(
					ResourceManager::GetTexture(normal)->get_mipLevels()),
				.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
				.unnormalizedCoordinates = false)

  // clang-format on

  CREATE_SAMPLER(diffuseSampler)
  CREATE_SAMPLER(normalSampler)

  descriptorSetInfo.samplers = {&diffuseSampler, &normalSampler};

  descriptorSet.init(descriptorSetInfo);
}

#undef DEFINE_SAMPLER
#undef CREATE_SAMPLER

Mesh::Submesh::~Submesh() {
  g_device.destroySampler(diffuseSampler, g_allocationCallbacks);
  g_device.destroySampler(normalSampler, g_allocationCallbacks);
}

void Mesh::draw_queue(const Queue &queue,
                      const vk::CommandBuffer &commandbuffer,
                      const Shader &shader) const {

  for (size_t i = 0; i < queue.size(); ++i) {
    commandbuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, shader.get_pipeline_layout(), 0, 1,
        &queue[i].descriptorSet.get_handle(), 0, nullptr);

    commandbuffer.pushConstants(
        shader.get_pipeline_layout(),
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
        sizeof(ModelInstancePushConstant::ModelInstanceData),
        sizeof(Eigen::Vector3f), &queue[i].diffuseColor);

    commandbuffer.drawIndexed(queue[i].indexCount, 1, queue[i].indexOffset, 0,
                              0);
  }
}

void Mesh::bind_buffer(const vk::CommandBuffer &commandbuffer,
                       const Shader &shader) const {

  commandbuffer.bindIndexBuffer(geometryInstance.buffer.get_handle(), 0,
                                VULKAN_INDICES_TYPE);
  commandbuffer.bindVertexBuffers(0, 1, &geometryInstance.buffer.get_handle(),
                                  &geometryInstance.vertexOffset);
}

// you must call bind_buffer before the first draw of the current update
void Mesh::draw(const vk::CommandBuffer &commandbuffer,
                const Shader &shader) const {

  draw_queue(opaqueQueue, commandbuffer, shader);
  draw_queue(transparentQueue, commandbuffer, shader);
}
} // namespace Renderer
