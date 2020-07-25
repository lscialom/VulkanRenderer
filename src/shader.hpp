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
  vk::SpecializationInfo *vertexSpecializationInfo = nullptr;
  vk::SpecializationInfo *fragmentSpecializationInfo = nullptr;

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

  void init_pipeline(
      const std::string &vertPath, const std::string &fragPath,
      const vk::RenderPass &renderPass, uint32_t nbColorAttachments,
      bool useVertexInput, bool cull, bool blendEnable,
      const std::vector<vk::DescriptorSetLayout> &uboLayouts,
      const std::vector<DescriptorLayout> &unmanagedDescriptors,
      const vk::SpecializationInfo *vertexSpecializationInfo = nullptr,
      const vk::SpecializationInfo *fragmentSpecializationInfo = nullptr);

public:
  Shader() = default;
  Shader(Shader &&other) { *this = std::move(other); }

  Shader &operator=(Shader &&other) {
    pipeline = other.pipeline;
    other.pipeline = std::nullptr_t(VK_NULL_HANDLE);

    pipelineLayout = other.pipelineLayout;
    other.pipelineLayout = std::nullptr_t(VK_NULL_HANDLE);

    descriptorSets = std::move(other.descriptorSets);
    other.descriptorSets.clear();

    ubos = other.ubos;
    other.ubos.clear();

    pushConstants = other.pushConstants;
    other.pushConstants.clear();

    return *this;
  }

  ~Shader() { destroy(); }

  const vk::PipelineLayout get_pipeline_layout() const;

  void bind_pipeline(const vk::CommandBuffer &commandbuffer) const;

  void init(const std::string &vertPath, const std::string &fragPath,
            const vk::RenderPass &renderPass, uint32_t nbColorAttachments,
            bool useVertexInput, bool cull, bool blendEnable,
            const std::vector<vk::PushConstantRange> &pcRanges = {},
            const std::vector<DescriptorSetInfo> &descriptors = {},
            const std::vector<UniformBufferObject *> &vUbo = {},
            const std::vector<DescriptorLayout> &unmanagedDescriptors = {},
            const vk::SpecializationInfo *vertexSpecializationInfo = nullptr,
            const vk::SpecializationInfo *fragmentSpecializationInfo = nullptr);

  void update_descriptors() const;

  void bind_descriptors(const vk::CommandBuffer &commandbuffer,
                        uint32_t offset = 0) const;

  void bind_pushConstants(const vk::CommandBuffer &commandbuffer,
                          const std::vector<void *> &data) const;

  void bind_ubos(const vk::CommandBuffer &commandbuffer,
                 uint32_t frameIndex) const;

  void bind_resources(const vk::CommandBuffer &commandbuffer,
                      uint32_t frameIndex,
                      const std::vector<void *> &pushConstantData = {}) const;

  void destroy();
};

} // namespace Renderer
