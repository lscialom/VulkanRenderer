#pragma once

#include "vulkan/vulkan.hpp"
#include "window_handler.hpp"

namespace Swapchain {
extern vk::PresentModeKHR PreferredPresentMode;
static constexpr vk::SurfaceFormatKHR PreferredImageFormat = {
    vk::Format::eR8G8B8A8Unorm,
    vk::ColorSpaceKHR::eSrgbNonlinear};

size_t ImageCount();
vk::AttachmentDescription GetAttachmentDescription();
vk::Extent2D GetExtent();

//TODO Both temporary need to move some of sync code directly into swapchain.cpp
const vk::SwapchainKHR& GetHandle();
vk::Framebuffer GetFramebuffer(size_t index);

void Init();
void InitFramebuffers(vk::RenderPass renderPass);

void Destroy();
} // namespace Swapchain
