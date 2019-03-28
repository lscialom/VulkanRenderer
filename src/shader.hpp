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

  bool useVertexInput;
  bool cull;
  bool blendEnable;
};

struct Shader {
private:
  vk::Pipeline pipeline;
  vk::PipelineLayout pipelineLayout;

  std::vector<DescriptorSet> descriptorSets;
  std::vector<vk::PushConstantRange> pushConstants;

  void init_descriptors(const std::vector<DescriptorSetInfo> &descriptors);

  void init_pipeline(const std::string &vertPath, const std::string &fragPath,
                     const vk::RenderPass &renderPass,
                     uint32_t nbColorAttachments, bool useVertexInput,
                     bool cull, bool blendEnable,
                     const std::vector<vk::DescriptorSetLayout> &uboLayouts);

public:
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
            const std::vector<vk::DescriptorSetLayout> &uboLayouts = {}) {

    pushConstants = pcRanges;

    init_descriptors(descriptors);
    init_pipeline(vertPath, fragPath, renderPass, nbColorAttachments,
                  useVertexInput, cull, blendEnable, uboLayouts);
  }

  void update_descriptors() const {
    for (size_t i = 0; i < descriptorSets.size(); ++i)
      descriptorSets[i].update();
  }

  void bind_descriptors(const vk::CommandBuffer &commandbuffer,
                        uint32_t offset = 0) const {
    for (size_t i = 0; i < descriptorSets.size(); ++i) {
      commandbuffer.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, pipelineLayout, offset + i, 1,
          &descriptorSets[i].get_handle(), 0, nullptr);
    }
  }

  void bind_pushConstants(const vk::CommandBuffer &commandbuffer,
                          const std::vector<void *> &data) const {

    assert(data.size() == pushConstants.size());

    for (size_t i = 0; i < pushConstants.size(); ++i) {
      commandbuffer.pushConstants(
          pipelineLayout,
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
          pushConstants[i].offset, pushConstants[i].size, data[i]);
    }
  }

  void destroy();
};

} // namespace Renderer
