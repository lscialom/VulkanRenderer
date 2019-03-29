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

    // See ShaderInfo struct for more details about this variable
    uint32_t drawRectCount;
  };

  std::vector<vk::Framebuffer> framebuffers;
  std::vector<Attachment> attachments;

  vk::RenderPass handle;

  std::vector<ShaderInternal> shaders;
  uint32_t nbColorAttachments = 0;

  vk::Extent2D extent;

  bool offscreen;

  bool attachmentsCleared = false;

  void init_render_pass_offscreen(const std::vector<AttachmentInfo> &attachmentInfos) {

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

    framebuffers.resize(1);

    // TODO Make one framebuffer per different attachment sizes
    CHECK_VK_RESULT_FATAL(g_device.createFramebuffer(&framebufferInfo,
                                                     g_allocationCallbacks,
                                                     &framebuffers[0]),
                          "Failed to create framebuffer.");
  }

  void init_shaders(const std::vector<ShaderInfo> &shaderInfos) {
    assert(!shaderInfos.empty());

    shaders.resize(shaderInfos.size());

    for (size_t i = 0; i < shaders.size(); ++i) {

      shaders[i].handle.init(shaderInfos[i].vertPath, shaderInfos[i].fragPath,
                             handle, nbColorAttachments,
                             shaderInfos[i].useVertexInput, shaderInfos[i].cull,
                             shaderInfos[i].blendEnable,
                             shaderInfos[i].pushConstants,
                             shaderInfos[i].descriptors, shaderInfos[i].ubos);

      shaders[i].drawModels = shaderInfos[i].drawModels;
      shaders[i].drawRectCount = shaderInfos[i].drawRectCount;
      shaders[i].hasPushConstants = !shaderInfos[i].pushConstants.empty();
    }
  }

public:
  ~Pass() { destroy(); }

  void init(const std::vector<ShaderInfo> &shaderInfos) {
    offscreen = false;
    extent = Swapchain::GetExtent();
    nbColorAttachments = 1;

    // Fourth Pass
    std::array<vk::AttachmentDescription, 1> attachmentDescriptions = {
        Swapchain::GetAttachmentDescription()};

    vk::AttachmentReference swapchainColorAttachmentRef(
        0, vk::ImageLayout::eColorAttachmentOptimal);

    vk::SubpassDescription subpass(vk::SubpassDescriptionFlags(),
                                   vk::PipelineBindPoint::eGraphics, 0, nullptr,
                                   1, &swapchainColorAttachmentRef);

    // vk::SubpassDependency dependency(
    //    VK_SUBPASS_EXTERNAL, 0,
    //    vk::PipelineStageFlagBits::eColorAttachmentOutput,
    //    vk::PipelineStageFlagBits::eColorAttachmentOutput,
    //    (vk::AccessFlagBits)0,
    //    vk::AccessFlagBits::eColorAttachmentRead |
    //        vk::AccessFlagBits::eColorAttachmentWrite);

    vk::RenderPassCreateInfo renderPassInfo(
        vk::RenderPassCreateFlags(),
        static_cast<uint32_t>(attachmentDescriptions.size()),
        attachmentDescriptions.data(), 1, &subpass, 0, nullptr);

    CHECK_VK_RESULT_FATAL(g_device.createRenderPass(
                              &renderPassInfo, g_allocationCallbacks, &handle),
                          "Failed to create render pass.");

    const std::vector<vk::ImageView> &attachments = Swapchain::GetImageViews();

    framebuffers.resize(Swapchain::ImageCount());
    for (size_t i = 0; i < framebuffers.size(); i++) {
      std::array<vk::ImageView, 1> iattachments = {attachments[i]};

      vk::FramebufferCreateInfo framebufferInfo(
          vk::FramebufferCreateFlags(), handle,
          static_cast<uint32_t>(iattachments.size()), iattachments.data(),
          extent.width, extent.height, 1);

      CHECK_VK_RESULT_FATAL(g_device.createFramebuffer(&framebufferInfo,
                                                       g_allocationCallbacks,
                                                       &framebuffers[i]),
                            "Failed to create framebuffer.");
    }

    init_shaders(shaderInfos);
  }

  // TODO Manage multiple attachment extents
  void init_offscreen(vk::Extent2D attachmentsExtent,
                      const std::vector<AttachmentInfo> &attachmentInfos,
                      const std::vector<ShaderInfo> &shaderInfos) {

    offscreen = true;

    extent = attachmentsExtent;
    if (extent == vk::Extent2D(0, 0))
      extent = Swapchain::GetExtent();

	init_render_pass_offscreen(attachmentInfos);
    init_shaders(shaderInfos);
  }

  void record(const vk::CommandBuffer &commandbuffer, uint32_t frameIndex,
              const std::vector<std::vector<void *>> &pushConstantData,
              const std::vector<vk::ClearValue> clearValues,
              const std::vector<Model *> &models = {}) {

    vk::RenderPassBeginInfo renderPassInfo;

    if (offscreen) {
      assert(clearValues.size() == attachments.size());

      renderPassInfo = vk::RenderPassBeginInfo(
          handle, framebuffers[0], {{0, 0}, extent},
          static_cast<uint32_t>(clearValues.size()), clearValues.data());
    } else {
      assert(clearValues.size() == 1);

      renderPassInfo = vk::RenderPassBeginInfo(
          handle, framebuffers[frameIndex], {{0, 0}, extent},
          static_cast<uint32_t>(clearValues.size()), clearValues.data());
    }

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
        commandbuffer.draw(6 * shaders[i].drawRectCount, 1, 0, 0);
    }

    commandbuffer.endRenderPass();

    attachmentsCleared = false;
  }

  void record_clear_attachments(const vk::CommandBuffer &commandbuffer,
                                vk::ClearValue clearValue) {

    if (attachmentsCleared)
      return;

    // TODO Optimize ?
    for (size_t i = 0; i < framebuffers.size(); ++i) {
      vk::RenderPassBeginInfo renderPassInfo(handle, framebuffers[i],
                                             {{0, 0}, extent}, 1, &clearValue);

      commandbuffer.beginRenderPass(&renderPassInfo,
                                    vk::SubpassContents::eInline);

      commandbuffer.endRenderPass();
    }

    attachmentsCleared = true;
  }

  const Image &get_attachment_image(size_t index = 0) const {
    return attachments[index].get_image();
  }

  void destroy() {

    extent = vk::Extent2D(0, 0);
    nbColorAttachments = 0;
    attachmentsCleared = false;

    if (!framebuffers.empty()) {
      for (size_t i = 0; i < framebuffers.size(); ++i) {
        g_device.destroyFramebuffer(framebuffers[i], g_allocationCallbacks);
        framebuffers[i] = nullptr;
      }

      framebuffers.clear();
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
