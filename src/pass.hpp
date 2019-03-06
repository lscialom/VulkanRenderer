#pragma once

#include <array>

namespace Renderer {
// TODO Array of layouts for Passes

#define DEFINE_LAYOUT(name, bindingCount, ...)                                 \
  static constexpr std::array<VkDescriptorSetLayoutBinding, bindingCount>      \
      name##LayoutInfo = {__VA_ARGS__};

#define DEFINE_LAYOUT_BINDING(...)                                             \
  { __VA_ARGS__ }

// clang-format off

DEFINE_LAYOUT(SSAO, 2,
	{
		// SSAO Noise Rotations
		DEFINE_LAYOUT_BINDING(
			.binding = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT
		),

		// SSAO Sample Kernel
        DEFINE_LAYOUT_BINDING(
			.binding = 1,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
			.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT
		),
	});

// clang-format on

#undef DEFINE_LAYOUT
#undef DEFINE_LAYOUT_BINDING

#define CREATE_DESCRIPTOR_SET_LAYOUT(layoutBindings, dstLayout)                \
  vk::DescriptorSetLayoutCreateInfo layoutBindings##layoutInfo(                \
      vk::DescriptorSetLayoutCreateFlags(),                                    \
      static_cast<uint32_t>(layoutBindings.size()),                            \
      reinterpret_cast<const vk::DescriptorSetLayoutBinding *>(                \
          layoutBindings.data()));                                             \
                                                                               \
  CHECK_VK_RESULT_FATAL(                                                       \
      g_device.createDescriptorSetLayout(&layoutBindings##layoutInfo,          \
                                         g_allocationCallbacks, &dstLayout),   \
      "Failed to create descriptor set layout.");

struct DescriptorSet {
private:
  // TODO Optimize
  vk::DescriptorPool pool;
  vk::DescriptorSet set;
  vk::DescriptorSetLayout layout;

public:
  void init(vk::DescriptorSetLayoutBinding *bindings, uint32_t bindingCount,
            Buffer *buffers, uint32_t bufferCount, Image *images,
            uint32_t imageCount) {

    vk::DescriptorSetLayoutCreateInfo layoutInfo(
        vk::DescriptorSetLayoutCreateFlags(), bindingCount, bindings);

    CHECK_VK_RESULT_FATAL(g_device.createDescriptorSetLayout(
                              &layoutInfo, g_allocationCallbacks, &layout),
                          "Failed to create descriptor set layout.");
  }
};

struct Shader {
private:
  // vk::DescriptorSetLayout descriptorSetLayout;

  vk::Pipeline pipeline;
  vk::PipelineLayout pipelineLayout;

  static vk::ShaderModule CreateShaderModule(const std::vector<char> &code) {
    vk::ShaderModuleCreateInfo createInfo(
        vk::ShaderModuleCreateFlags(), code.size(),
        reinterpret_cast<const uint32_t *>(code.data()));

    vk::ShaderModule shaderModule;
    CHECK_VK_RESULT_FATAL(g_device.createShaderModule(&createInfo,
                                                      g_allocationCallbacks,
                                                      &shaderModule),
                          "Failed to create shader module");

    return shaderModule;
  }

  void init_pipeline(const std::string &vertPath, const std::string &fragPath,
                     const vk::RenderPass &renderPass,
                     uint32_t nbColorAttachments, bool useVertexInput,
                     bool cull, bool blendEnable,
                     const std::vector<vk::PushConstantRange> &pushConstants,
                     const std::vector<vk::DescriptorSetLayout> &descriptors) {
    auto vertShaderCode = ReadFile(vertPath);
    auto fragShaderCode = ReadFile(fragPath);

    vk::ShaderModule vertShaderModule = CreateShaderModule(vertShaderCode);
    vk::ShaderModule fragShaderModule = CreateShaderModule(fragShaderCode);

    vk::PipelineShaderStageCreateInfo vertShaderStageInfo(
        vk::PipelineShaderStageCreateFlags(), vk::ShaderStageFlagBits::eVertex,
        vertShaderModule, "main");

    vk::PipelineShaderStageCreateInfo fragShaderStageInfo(
        vk::PipelineShaderStageCreateFlags(),
        vk::ShaderStageFlagBits::eFragment, fragShaderModule, "main");

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
    vk::Viewport viewport(0, 0, (float)extent.width, (float)extent.height, 0,
                          1);

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

    std::array<vk::DynamicState, 2> dynamicStates = {
        vk::DynamicState::eViewport, vk::DynamicState::eScissor};
    vk::PipelineDynamicStateCreateInfo dynamicStateCreateInfo(
        vk::PipelineDynamicStateCreateFlags(),
        static_cast<uint32_t>(dynamicStates.size()), dynamicStates.data());

    vk::PipelineLayoutCreateInfo pipelineLayoutInfo =
        vk::PipelineLayoutCreateInfo(
            vk::PipelineLayoutCreateFlags(),
            static_cast<uint32_t>(descriptors.size()), descriptors.data(),
            static_cast<uint32_t>(pushConstants.size()), pushConstants.data());

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
            const std::vector<vk::DescriptorSetLayout> &descriptors = {}) {

    init_pipeline(vertPath, fragPath, renderPass, nbColorAttachments,
                  useVertexInput, cull, blendEnable, pushConstants,
                  descriptors);
  }

  void destroy() {
    g_device.destroyPipeline(pipeline, g_allocationCallbacks);
    g_device.destroyPipelineLayout(pipelineLayout, g_allocationCallbacks);
  }
};

template <size_t NumShaders, size_t NumAttachments> struct Pass {
private:
  vk::Framebuffer framebuffer;
  vk::RenderPass handle;

  std::array<Shader, NumShaders> shaders;
  std::array<Attachment, NumAttachments> attachments;
};
} // namespace Renderer