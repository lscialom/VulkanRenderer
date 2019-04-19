#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "descriptors.hpp"

namespace Renderer {

struct ShaderInfo {
  std::string vertPath;
  std::string fragPath;
  std::vector<vk::PushConstantRange> pushConstants;
  std::vector<DescriptorSetInfo> descriptors;
  std::vector<UniformBufferObject *> ubos;

  // For offscreen. Multiplier on number of vertices to draw. So 1 means 6 * 1
  // vertices drawn (1 rect)
  uint32_t drawRectCount = 1;

  bool useVertexInput;
  bool cull;
  bool blendEnable;
  bool drawModels;
};

struct Shader {
private:
  vk::Pipeline pipeline;
  vk::PipelineLayout pipelineLayout;

  std::vector<DescriptorSet> descriptorSets;
  std::vector<UniformBufferObject *> ubos;
  std::vector<vk::PushConstantRange> pushConstants;

  size_t unmanagedDescriptorsCount;

  void init_descriptors(const std::vector<DescriptorSetInfo> &descriptors);

  void init_pipeline(const std::string &vertPath, const std::string &fragPath,
                     const vk::RenderPass &renderPass,
                     uint32_t nbColorAttachments, bool useVertexInput,
                     bool cull, bool blendEnable,
                     const std::vector<vk::DescriptorSetLayout> &uboLayouts,
                     const std::vector<DescriptorLayout> &unmanagedDescriptors);

public:
  ~Shader() { destroy(); }

  const vk::PipelineLayout get_pipeline_layout() const {
    return pipelineLayout;
  }

  void bind_pipeline(const vk::CommandBuffer &commandbuffer) const {
    commandbuffer.bindPipeline(
        vk::PipelineBindPoint::eGraphics, // TODO Compute shader support
        pipeline);
  }

  void init(const std::string &vertPath, const std::string &fragPath,
            const vk::RenderPass &renderPass, uint32_t nbColorAttachments,
            bool useVertexInput, bool cull, bool blendEnable,
            const std::vector<vk::PushConstantRange> &pcRanges = {},
            const std::vector<DescriptorSetInfo> &descriptors = {},
            const std::vector<UniformBufferObject *> &vUbo = {},
            const std::vector<DescriptorLayout> &unmanagedDescriptors = {}) {

    pushConstants = pcRanges;
    ubos = vUbo;

    unmanagedDescriptorsCount = unmanagedDescriptors.size();

    std::vector<vk::DescriptorSetLayout> uboLayouts;
    uboLayouts.resize(ubos.size());

    for (size_t i = 0; i < uboLayouts.size(); ++i)
      uboLayouts[i] = ubos[i]->get_layout();

    init_descriptors(descriptors);
    init_pipeline(vertPath, fragPath, renderPass, nbColorAttachments,
                  useVertexInput, cull, blendEnable, uboLayouts,
                  unmanagedDescriptors);
  }

  void update_descriptors() const {
    for (size_t i = 0; i < descriptorSets.size(); ++i)
      descriptorSets[i].update();
  }

  void bind_descriptors(const vk::CommandBuffer &commandbuffer,
                        uint32_t offset = 0) const {
    for (size_t i = 0; i < descriptorSets.size(); ++i) {
      commandbuffer.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, pipelineLayout, offset + i + unmanagedDescriptorsCount, 1,
          &descriptorSets[i].get_handle(), 0, nullptr);
    }
  }

  void bind_pushConstants(const vk::CommandBuffer &commandbuffer,
                          const std::vector<void *> &data) const {

    assert(data.size() <= pushConstants.size());

    for (size_t i = 0; i < data.size(); ++i) {
      commandbuffer.pushConstants(
          pipelineLayout,
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
          pushConstants[i].offset, pushConstants[i].size, data[i]);
    }
  }

  void bind_ubos(const vk::CommandBuffer &commandbuffer,
                 uint32_t frameIndex) const {

    for (size_t i = 0; i < ubos.size(); ++i) {

      commandbuffer.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, pipelineLayout, i + unmanagedDescriptorsCount, 1,
          &ubos[i]->get_descriptor_set(frameIndex), 0, nullptr);
    }
  }

  void bind_resources(const vk::CommandBuffer &commandbuffer,
                      uint32_t frameIndex,
                      const std::vector<void *> &pushConstantData = {}) const {

    bind_ubos(commandbuffer, frameIndex);
    bind_descriptors(commandbuffer, ubos.size());
    bind_pushConstants(commandbuffer, pushConstantData);
  }

  void destroy();
};

} // namespace Renderer
