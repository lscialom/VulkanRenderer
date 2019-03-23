#pragma once

#include "shader.hpp"

namespace Renderer {

struct Pass {
private:
  vk::Framebuffer framebuffer;
  std::vector<Attachment> attachments;

  vk::RenderPass handle;

  std::vector<Shader> shaders;

public:
  void init(const std::vector<DescriptorLayout> &descriptorLayouts) {}
};
} // namespace Renderer
