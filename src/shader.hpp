#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "descriptors.hpp"

namespace Renderer {

struct Shader {
private:
  vk::Pipeline pipeline;
  vk::PipelineLayout pipelineLayout;

  std::vector<DescriptorSet> descriptorSets;

  void init_descriptors(const std::vector<DescriptorSetInfo> &descriptors);

  void init_pipeline(const std::string &vertPath, const std::string &fragPath,
                     const vk::RenderPass &renderPass,
                     uint32_t nbColorAttachments, bool useVertexInput,
                     bool cull, bool blendEnable,
                     const std::vector<vk::PushConstantRange> &pushConstants,
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
            const std::vector<vk::PushConstantRange> &pushConstants = {},
            const std::vector<DescriptorSetInfo> &descriptors = {},
            const std::vector<vk::DescriptorSetLayout> &uboLayouts = {}) {

    init_descriptors(descriptors);
    init_pipeline(vertPath, fragPath, renderPass, nbColorAttachments,
                  useVertexInput, cull, blendEnable, pushConstants, uboLayouts);
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

  void destroy();
};

} // namespace Renderer
