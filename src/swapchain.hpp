#pragma once

#include "global_context.hpp"

namespace Renderer {
namespace Swapchain {

extern vk::PresentModeKHR PreferredPresentMode;
static inline vk::SurfaceFormatKHR PreferredImageFormat{
    vk::Format::eB8G8R8A8Srgb, vk::ColorSpaceKHR::eSrgbNonlinear};

size_t ImageCount();
uint32_t GetCurrentImageIndex();
const vk::Semaphore *GetCurrentImageSemaphore();

size_t GetCurrentFrameIndex();
vk::Fence GetCurrentFrameFence();

vk::AttachmentDescription GetAttachmentDescription();
vk::Extent2D GetExtent();

const std::vector<vk::ImageView> &GetImageViews();

void SetPresentQueue(Queue queue);

void Init(uint32_t w, uint32_t h);

void Destroy();

vk::Result AcquireNextImage();
vk::Result Present(const vk::Semaphore *presentSemaphore);

} // namespace Swapchain
} // namespace Renderer
