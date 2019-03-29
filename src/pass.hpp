#pragma once

#include "shader.hpp"

#include "model.hpp"

namespace Renderer {

struct Pass {
private:
  struct ShaderInternal {
    Shader handle;
    bool drawModels;
    bool hasPushConstants;
  };

  vk::Framebuffer framebuffer;
  std::vector<Attachment> attachments;

  vk::RenderPass handle;

  std::vector<ShaderInternal> shaders;
  uint32_t nbColorAttachments = 0;

  vk::Extent2D extent;

  bool attachmentsCleared = false;

  void init_render_pass(const std::vector<AttachmentInfo> &attachmentInfos) {

    std::vector<vk::ImageView> imageViews;

    std::vector<vk::AttachmentDescription> attachmentDescriptions;
    std::vector<vk::AttachmentReference> colorRefs;
    vk::AttachmentReference depthAttachmentRef;

    uint8_t depthCount = 0;

    attachments.resize(attachmentInfos.size());
    for (size_t i = 0; i < attachments.size(); ++i) {

      attachments[i].requiredFormat = attachmentInfos[i].format;

      if (attachmentInfos[i].isDepth) {

        assert(++depthCount == 1);

        attachments[i].init(
            extent.width, extent.height, vk::ImageTiling::eOptimal,
            vk::ImageUsageFlagBits::eDepthStencilAttachment |
                vk::ImageUsageFlagBits::eSampled,
            vk::MemoryPropertyFlagBits::eDeviceLocal,
            vk::ImageAspectFlagBits::eDepth,
            vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal);

        attachmentDescriptions.push_back(
            attachments[i].make_attachment_description(
                vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal,
                vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal));

        depthAttachmentRef = {static_cast<uint32_t>(i),
                              vk::ImageLayout::eDepthStencilAttachmentOptimal};

      } else {

        attachments[i].init(extent.width, extent.height);

        attachmentDescriptions.push_back(
            attachments[i].make_attachment_description(
                vk::ImageLayout::eShaderReadOnlyOptimal,
                vk::ImageLayout::eShaderReadOnlyOptimal));

        colorRefs.push_back({static_cast<uint32_t>(i),
                             vk::ImageLayout::eColorAttachmentOptimal});

        nbColorAttachments++;
      }

      imageViews.push_back(attachments[i].get_image_view());
    }

    vk::SubpassDescription subpass(
        vk::SubpassDescriptionFlags(), vk::PipelineBindPoint::eGraphics, 0,
        nullptr, static_cast<uint32_t>(colorRefs.size()), colorRefs.data(),
        nullptr, (depthCount ? &depthAttachmentRef : nullptr));

    vk::RenderPassCreateInfo renderPassInfo(
        vk::RenderPassCreateFlags(),
        static_cast<uint32_t>(attachmentDescriptions.size()),
        attachmentDescriptions.data(), 1, &subpass, 0, nullptr);

    CHECK_VK_RESULT_FATAL(g_device.createRenderPass(
                              &renderPassInfo, g_allocationCallbacks, &handle),
                          "Failed to create render pass.");

    vk::FramebufferCreateInfo framebufferInfo(
        vk::FramebufferCreateFlags(), handle,
        static_cast<uint32_t>(imageViews.size()), imageViews.data(),
        extent.width, extent.height, 1);

    // TODO Make one framebuffer per different attachment sizes
    CHECK_VK_RESULT_FATAL(g_device.createFramebuffer(&framebufferInfo,
                                                     g_allocationCallbacks,
                                                     &framebuffer),
                          "Failed to create framebuffer.");
  }

  void init_shaders(const std::vector<ShaderInfo> &shaderInfos) {
    shaders.resize(shaderInfos.size());

    for (size_t i = 0; i < shaders.size(); ++i) {

      shaders[i].handle.init(shaderInfos[i].vertPath, shaderInfos[i].fragPath,
                             handle, nbColorAttachments,
                             shaderInfos[i].useVertexInput, shaderInfos[i].cull,
                             shaderInfos[i].blendEnable,
                             shaderInfos[i].pushConstants,
                             shaderInfos[i].descriptors, shaderInfos[i].ubos);

      shaders[i].drawModels = shaderInfos[i].drawModels;
      shaders[i].hasPushConstants = !shaderInfos[i].pushConstants.empty();
    }
  }

public:
  ~Pass() { destroy(); }

  // TODO Manage multiple attachment extents
  void init(vk::Extent2D attachmentsExtent,
            const std::vector<AttachmentInfo> &attachmentInfos,
            const std::vector<ShaderInfo> &shaderInfos) {

    extent = attachmentsExtent;
    if (extent == vk::Extent2D(0, 0))
      extent = Swapchain::GetExtent();

    init_render_pass(attachmentInfos);
    init_shaders(shaderInfos);
  }

  void record(const vk::CommandBuffer &commandbuffer, uint32_t frameIndex,
              const std::vector<std::vector<void *>> &pushConstantData,
              const std::vector<vk::ClearValue> clearValues,
              const std::vector<Model *> &models = {}) {

    assert(clearValues.size() == attachments.size());

    vk::RenderPassBeginInfo renderPassInfo(
        handle, framebuffer, {{0, 0}, extent},
        static_cast<uint32_t>(clearValues.size()), clearValues.data());

    commandbuffer.beginRenderPass(&renderPassInfo,
                                  vk::SubpassContents::eInline);

    size_t currentPushConstantDataIndex = 0;
    for (size_t i = 0; i < shaders.size(); ++i) {

      shaders[i].handle.bind_pipeline(commandbuffer);

      // Auto push constant if have some and that it is not models push
      // constants (because they change at each draw calls)
      if (shaders[i].hasPushConstants && !shaders[i].drawModels) {
        shaders[i].handle.bind_resources(
            commandbuffer, frameIndex,
            pushConstantData[currentPushConstantDataIndex++]);
      } else {
        shaders[i].handle.bind_resources(commandbuffer, frameIndex);
      }

      if (shaders[i].drawModels) {

        // TODO Call a function from renderContext instead + move it in
        // global_context
        for (size_t j = 0; j < models.size(); ++j)
          models[j]->record(commandbuffer, shaders[i].handle, frameIndex);

      } else
        commandbuffer.draw(6, 1, 0, 0);
    }

    commandbuffer.endRenderPass();

    attachmentsCleared = false;
  }

  void record_clear_attachments(const vk::CommandBuffer &commandbuffer,
                                vk::ClearValue clearValue, size_t index = 0) {

    if (attachmentsCleared)
      return;

    vk::RenderPassBeginInfo renderPassInfo(handle, framebuffer,
                                           {{0, 0}, extent}, 1, &clearValue);

    commandbuffer.beginRenderPass(&renderPassInfo,
                                  vk::SubpassContents::eInline);

    commandbuffer.endRenderPass();

    attachmentsCleared = true;
  }

  const Image &get_attachment_image(size_t index = 0) const {
    return attachments[index].get_image();
  }

  void destroy() {

    extent = vk::Extent2D(0, 0);
    nbColorAttachments = 0;
    attachmentsCleared = false;

    if (framebuffer) {
      g_device.destroyFramebuffer(framebuffer, g_allocationCallbacks);
      framebuffer = nullptr;
    }

    attachments.clear();

    shaders.clear();

    if (handle) {
      g_device.destroyRenderPass(handle, g_allocationCallbacks);
      handle = nullptr;
    }
  }
};
} // namespace Renderer
