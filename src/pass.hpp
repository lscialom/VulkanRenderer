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

    std::string name;

    // See ShaderInfo struct for more details about this variable
    uint32_t drawRectCount;

    bool enabled = true;
  };

  std::vector<vk::Framebuffer> framebuffers;
  std::vector<Attachment> attachments;

  vk::RenderPass handle;

  std::vector<ShaderInternal> shaders;
  uint32_t nbColorAttachments = 0;

  vk::Extent2D extent;

  bool offscreen;

  bool attachmentsCleared = false;

  void init_render_pass_offscreen(
      const std::vector<AttachmentInfo> &attachmentInfos);

  void init_shaders(const std::vector<ShaderInfo> &shaderInfos);

public:
  ~Pass() { destroy(); }

  void init(const std::vector<ShaderInfo> &shaderInfos);

  // TODO Manage multiple attachment extents
  void init_offscreen(vk::Extent2D attachmentsExtent,
                      const std::vector<AttachmentInfo> &attachmentInfos,
                      const std::vector<ShaderInfo> &shaderInfos);

  void record(const vk::CommandBuffer &commandbuffer, uint32_t frameIndex,
              const std::vector<std::vector<void *>> &pushConstantData,
              const std::vector<vk::ClearValue> &clearValues,
              const std::vector<Model *> &models = {});

  void record_clear_attachments(const vk::CommandBuffer &commandbuffer,
                                vk::ClearValue clearValue);

  void set_shader_enabled(const std::string &shaderName, bool shouldEnable);

  const Image &get_attachment_image(size_t index = 0) const;

  void destroy();
};
} // namespace Renderer
