#pragma once

#include "shader.hpp"

namespace Renderer {

struct Pass {
private:
  vk::Framebuffer framebuffer;
  std::vector<Attachment> attachments;

  vk::RenderPass handle;

  std::vector<Shader> shaders;
  uint32_t nbColorAttachments = 0;

  void init_render_pass(vk::Extent2D extent,
                        const std::vector<AttachmentInfo> &attachmentInfos) {

    if (extent == vk::Extent2D(0, 0))
      extent = Swapchain::GetExtent();

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

        depthAttachmentRef = {i,
                              vk::ImageLayout::eDepthStencilAttachmentOptimal};

      } else {

        attachments[i].init(extent.width, extent.height);

        attachmentDescriptions.push_back(
            attachments[i].make_attachment_description(
                vk::ImageLayout::eShaderReadOnlyOptimal,
                vk::ImageLayout::eShaderReadOnlyOptimal));

        colorRefs.push_back({i, vk::ImageLayout::eColorAttachmentOptimal});

        nbColorAttachments++;
      }

      imageViews.push_back(attachments[i].get_image_view());
    }

    vk::SubpassDescription subpass(
        vk::SubpassDescriptionFlags(), vk::PipelineBindPoint::eGraphics, 0,
        nullptr, static_cast<uint32_t>(colorRefs.size()), colorRefs.data(),
        nullptr, &depthAttachmentRef);

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
      shaders[i].init(shaderInfos[i].vertPath, shaderInfos[i].fragPath, handle,
                      nbColorAttachments, shaderInfos[i].useVertexInput,
                      shaderInfos[i].cull, shaderInfos[i].blendEnable,
                      shaderInfos[i].pushConstants,
                      shaderInfos[i].descriptors, );
    }
  }

public:
  // TODO Manage multiple attachment extents
  void init(vk::Extent2D extent,
            const std::vector<AttachmentInfo> &attachmentInfos,
            const std::vector<ShaderInfo> &shaderInfos) {

    init_render_pass(extent, attachmentInfos);
    init_shaders(shaderInfos);
  }
};
} // namespace Renderer
