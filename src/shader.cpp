#include "shader.hpp"

#include "data_structures.hpp"
#include "debug_tools.hpp"
#include "global_context.hpp"
#include "swapchain.hpp"

#include <fstream>
#include <iostream>

static std::vector<char> ReadFile(const std::string &filename) {
  std::ifstream file(filename, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "[ERROR] "
              << "Could not open file " << filename << std::endl;
    return {};
  }

  size_t fileSize = (size_t)file.tellg();
  std::vector<char> buffer(fileSize);

  file.seekg(0);
  file.read(buffer.data(), fileSize);

  file.close();

  return buffer;
}

namespace Renderer {

static vk::ShaderModule CreateShaderModule(const std::vector<char> &code) {
  vk::ShaderModuleCreateInfo createInfo(
      vk::ShaderModuleCreateFlags(), code.size(),
      reinterpret_cast<const uint32_t *>(code.data()));

  vk::ShaderModule shaderModule;
  CHECK_VK_RESULT_FATAL(g_device.createShaderModule(
                            &createInfo, g_allocationCallbacks, &shaderModule),
                        "Failed to create shader module");

  return shaderModule;
}

void Shader::init_descriptors(
    const std::vector<DescriptorSetInfo> &descriptors) {

  descriptorSets.resize(descriptors.size());

  for (size_t i = 0; i < descriptorSets.size(); ++i)
    descriptorSets[i].init(descriptors[i], true);
}

void Shader::init_pipeline(
    const std::string &vertPath, const std::string &fragPath,
    const vk::RenderPass &renderPass, uint32_t nbColorAttachments,
    bool useVertexInput, bool cull, bool blendEnable,
    const std::vector<vk::DescriptorSetLayout> &uboLayouts,
    const std::vector<DescriptorLayout> &unmanagedDescriptors,
    const vk::SpecializationInfo *vertexSpecializationInfo,
    const vk::SpecializationInfo *fragmentSpecializationInfo) {

  auto vertShaderCode = ReadFile(vertPath);
  auto fragShaderCode = ReadFile(fragPath);

  vk::ShaderModule vertShaderModule = CreateShaderModule(vertShaderCode);
  vk::ShaderModule fragShaderModule = CreateShaderModule(fragShaderCode);

  vk::PipelineShaderStageCreateInfo vertShaderStageInfo(
      vk::PipelineShaderStageCreateFlags(), vk::ShaderStageFlagBits::eVertex,
      vertShaderModule, "main", vertexSpecializationInfo);

  vk::PipelineShaderStageCreateInfo fragShaderStageInfo(
      vk::PipelineShaderStageCreateFlags(), vk::ShaderStageFlagBits::eFragment,
      fragShaderModule, "main", fragmentSpecializationInfo);

  vk::PipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo,
                                                      fragShaderStageInfo};

  vk::VertexInputBindingDescription bindingDescription =
      LiteralVertex::GetBindingDescription();
  auto attributeDescriptions = LiteralVertex::GetAttributeDescription();

  vk::PipelineVertexInputStateCreateInfo vertexInputInfo;
  if (useVertexInput) {
    vertexInputInfo = vk::PipelineVertexInputStateCreateInfo(
        vk::PipelineVertexInputStateCreateFlags(), 1, &bindingDescription,
        static_cast<uint32_t>(attributeDescriptions.size()),
        reinterpret_cast<vk::VertexInputAttributeDescription *>(
            attributeDescriptions.data()));
  } else {
    vertexInputInfo = vk::PipelineVertexInputStateCreateInfo(
        vk::PipelineVertexInputStateCreateFlags(), 0, nullptr, 0, nullptr);
  }

  vk::PipelineInputAssemblyStateCreateInfo inputAssembly(
      vk::PipelineInputAssemblyStateCreateFlags(),
      vk::PrimitiveTopology::eTriangleList, VK_FALSE);

  vk::Extent2D extent = Swapchain::GetExtent();
  vk::Viewport viewport(0, 0, (float)extent.width, (float)extent.height, 0, 1);

  vk::Rect2D scissor({0, 0}, extent);

  vk::PipelineViewportStateCreateInfo viewportState(
      vk::PipelineViewportStateCreateFlags(), 1, &viewport, 1, &scissor);

  vk::CullModeFlagBits cullMode =
      cull ? vk::CullModeFlagBits::eBack : vk::CullModeFlagBits::eNone;

  vk::PipelineRasterizationStateCreateInfo rasterizer(
      vk::PipelineRasterizationStateCreateFlags(), VK_FALSE, VK_FALSE,
      vk::PolygonMode::eFill, cullMode, vk::FrontFace::eCounterClockwise,
      VK_FALSE, 0, 0, 0, 1);

  vk::PipelineMultisampleStateCreateInfo multisampling(
      vk::PipelineMultisampleStateCreateFlags(), vk::SampleCountFlagBits::e1,
      VK_FALSE, 1, nullptr, VK_FALSE, VK_FALSE);

  vk::PipelineDepthStencilStateCreateInfo depthStencil(
      vk::PipelineDepthStencilStateCreateFlags(), true, true,
      vk::CompareOp::eLess, false, false, vk::StencilOpState(),
      vk::StencilOpState(), 0.0f, 1.0f);

  vk::PipelineColorBlendAttachmentState colorBlendAttachment(
      blendEnable, vk::BlendFactor::eSrcAlpha,
      vk::BlendFactor::eOneMinusSrcAlpha, vk::BlendOp::eAdd,
      vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
      vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
          vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

  std::vector<vk::PipelineColorBlendAttachmentState> colorBlendAttachments(
      nbColorAttachments, colorBlendAttachment);

  vk::PipelineColorBlendStateCreateInfo colorBlending(
      vk::PipelineColorBlendStateCreateFlags(), VK_FALSE, vk::LogicOp::eCopy,
      static_cast<uint32_t>(colorBlendAttachments.size()),
      colorBlendAttachments.data());

  std::array<vk::DynamicState, 2> dynamicStates = {vk::DynamicState::eViewport,
                                                   vk::DynamicState::eScissor};
  vk::PipelineDynamicStateCreateInfo dynamicStateCreateInfo(
      vk::PipelineDynamicStateCreateFlags(),
      static_cast<uint32_t>(dynamicStates.size()), dynamicStates.data());

  std::vector<vk::DescriptorSetLayout> descriptorLayouts;

  for (size_t i = 0; i < unmanagedDescriptors.size(); ++i)
    descriptorLayouts.push_back(unmanagedDescriptors[i].handle);

  for (size_t i = 0; i < uboLayouts.size(); ++i)
    descriptorLayouts.push_back(uboLayouts[i]);

  for (size_t i = 0; i < descriptorSets.size(); ++i)
    descriptorLayouts.push_back(descriptorSets[i].get_layout());

  vk::PipelineLayoutCreateInfo pipelineLayoutInfo =
      vk::PipelineLayoutCreateInfo(
          vk::PipelineLayoutCreateFlags(),
          static_cast<uint32_t>(descriptorLayouts.size()),
          descriptorLayouts.data(), static_cast<uint32_t>(pushConstants.size()),
          pushConstants.data());

  CHECK_VK_RESULT_FATAL(g_device.createPipelineLayout(&pipelineLayoutInfo,
                                                      g_allocationCallbacks,
                                                      &pipelineLayout),
                        "Failed to create pipeline layout.");

  vk::GraphicsPipelineCreateInfo pipelineInfo(
      vk::PipelineCreateFlags(), 2, shaderStages, &vertexInputInfo,
      &inputAssembly, nullptr, &viewportState, &rasterizer, &multisampling,
      &depthStencil, &colorBlending, &dynamicStateCreateInfo, pipelineLayout,
      renderPass, 0, nullptr, -1);

  CHECK_VK_RESULT_FATAL(
      g_device.createGraphicsPipelines(nullptr, 1, &pipelineInfo,
                                       g_allocationCallbacks, &pipeline),
      "Failed to create pipeline layout.");

  g_device.destroyShaderModule(fragShaderModule, g_allocationCallbacks);
  g_device.destroyShaderModule(vertShaderModule, g_allocationCallbacks);
}

void Shader::destroy() {
  if (pipeline) {
    g_device.destroyPipeline(pipeline, g_allocationCallbacks);
    pipeline = nullptr;
  }

  if (pipelineLayout) {
    g_device.destroyPipelineLayout(pipelineLayout, g_allocationCallbacks);
    pipelineLayout = nullptr;
  }

  descriptorSets.clear();
  ubos.clear();
  pushConstants.clear();
}

const vk::PipelineLayout Shader::get_pipeline_layout() const {
  return pipelineLayout;
}

void Shader::bind_pipeline(const vk::CommandBuffer &commandbuffer) const {
  commandbuffer.bindPipeline(
      vk::PipelineBindPoint::eGraphics, // TODO Compute shader support
      pipeline);
}

void Shader::init(const std::string &vertPath, const std::string &fragPath,
                  const vk::RenderPass &renderPass, uint32_t nbColorAttachments,
                  bool useVertexInput, bool cull, bool blendEnable,
                  const std::vector<vk::PushConstantRange> &pcRanges,
                  const std::vector<DescriptorSetInfo> &descriptors,
                  const std::vector<UniformBufferObject *> &vUbo,
                  const std::vector<DescriptorLayout> &unmanagedDescriptors,
                  const vk::SpecializationInfo *vertexSpecializationInfo,
                  const vk::SpecializationInfo *fragmentSpecializationInfo) {

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
                unmanagedDescriptors, vertexSpecializationInfo,
                fragmentSpecializationInfo);
}

void Shader::update_descriptors() const {
  for (size_t i = 0; i < descriptorSets.size(); ++i)
    descriptorSets[i].update();
}

void Shader::bind_descriptors(const vk::CommandBuffer &commandbuffer,
                              uint32_t offset) const {
  for (size_t i = 0; i < descriptorSets.size(); ++i) {
    commandbuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, pipelineLayout,
        offset + i + unmanagedDescriptorsCount, 1,
        &descriptorSets[i].get_handle(), 0, nullptr);
  }
}

void Shader::bind_pushConstants(const vk::CommandBuffer &commandbuffer,
                                const std::vector<void *> &data) const {

  assert(data.size() <= pushConstants.size());

  for (size_t i = 0; i < data.size(); ++i) {
    commandbuffer.pushConstants(
        pipelineLayout,
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
        pushConstants[i].offset, pushConstants[i].size, data[i]);
  }
}

void Shader::bind_ubos(const vk::CommandBuffer &commandbuffer,
                       uint32_t frameIndex) const {

  for (size_t i = 0; i < ubos.size(); ++i) {

    commandbuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, pipelineLayout,
        i + unmanagedDescriptorsCount, 1,
        &ubos[i]->get_descriptor_set(frameIndex), 0, nullptr);
  }
}

void Shader::bind_resources(const vk::CommandBuffer &commandbuffer,
                            uint32_t frameIndex,
                            const std::vector<void *> &pushConstantData) const {

  bind_ubos(commandbuffer, frameIndex);
  bind_descriptors(commandbuffer, ubos.size());
  bind_pushConstants(commandbuffer, pushConstantData);
}

} // namespace Renderer
