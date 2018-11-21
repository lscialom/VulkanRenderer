#pragma once

#include "vulkan/vulkan.hpp"
#include "window_handler.hpp"

namespace Swapchain {
extern vk::PresentModeKHR PreferredPresentMode;
static constexpr vk::SurfaceFormatKHR PreferredImageFormat = {
    vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear};

size_t ImageCount();
uint32_t GetCurrentImageIndex();
const vk::Semaphore *GetCurrentImageSemaphore();

size_t GetCurrentFrameIndex();
vk::Fence GetCurrentFrameFence();

vk::AttachmentDescription GetAttachmentDescription();
vk::Extent2D GetExtent();

// TODO Add/Replace for something like GetCurrentFramebuffer() ? Probably not
// since Swapchain don't have to sync this with its other stuff. So it's
// Renderer's job (since it's Renderer's commandbuffers that write on these
// framebuffers) ?
// ANSWER => Or just move the framebuffers back to Renderer. Framebuffers are
// tied to the images but it's the commandbuffers who decide which one to use
// and the signal semaphore attached to the submit command who gives the signal
// for the presentation of the resulting image so Renderer should have total
// control on them.
vk::Framebuffer GetFramebuffer(size_t index);

void SetPresentQueue(struct Queue queue);

void Init();

// Recreates framebuffers with renderPass if already initialized
void InitFramebuffers(vk::RenderPass renderPass);

void Destroy();

vk::Result AcquireNextImage();
vk::Result Present(const vk::Semaphore *presentSemaphore);
} // namespace Swapchain