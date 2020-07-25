#include "swapchain.hpp"

#include "configuration_helper.hpp"

namespace Renderer {

namespace Swapchain {
vk::PresentModeKHR PreferredPresentMode = vk::PresentModeKHR::eMailbox;

static vk::Format imageFormat;
static vk::Extent2D extent = {0, 0};

static vk::SwapchainKHR handle;

static std::vector<vk::Image> images;
static std::vector<vk::ImageView> imageViews;

static std::array<vk::Semaphore, MAX_IN_FLIGHT_FRAMES> imageAvailableSemaphores;
static std::array<vk::Fence, MAX_IN_FLIGHT_FRAMES> inFlightFences;

static size_t currentFrame = 0;
static uint32_t currentImageIndex = 0;

static Queue presentQueue;

vk::AttachmentDescription GetAttachmentDescription() {
  return vk::AttachmentDescription(
      vk::AttachmentDescriptionFlags(), imageFormat,
      vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
      vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
      vk::AttachmentStoreOp::eDontCare,
      vk::ImageLayout::eUndefined, // TODO transition swapchain images to
                                   // present
      vk::ImageLayout::ePresentSrcKHR);
}

size_t ImageCount() { return images.size(); }
uint32_t GetCurrentImageIndex() { return currentImageIndex; }
const vk::Semaphore *GetCurrentImageSemaphore() {
  return &imageAvailableSemaphores[currentFrame];
}

size_t GetCurrentFrameIndex() { return currentFrame; }
vk::Fence GetCurrentFrameFence() { return inFlightFences[currentFrame]; }

vk::Extent2D GetExtent() { return extent; }

std::array<float, 2> GetExtentF() {
  return {
      {static_cast<float>(extent.width), static_cast<float>(extent.height)}};
}

const std::vector<vk::ImageView> &GetImageViews() { return imageViews; }

void SetPresentQueue(Queue queue) { presentQueue = queue; }

static void InitSyncBarriers() {
  vk::SemaphoreCreateInfo semaphoreInfo = {};

  vk::FenceCreateInfo fenceInfo = {vk::FenceCreateFlagBits::eSignaled};

  for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++) {
    CHECK_VK_RESULT_FATAL(
        g_device.createSemaphore(&semaphoreInfo, g_allocationCallbacks,
                                 &imageAvailableSemaphores[i]),
        "Failed to create image available semaphore.");

    CHECK_VK_RESULT_FATAL(g_device.createFence(&fenceInfo,
                                               g_allocationCallbacks,
                                               &inFlightFences[i]),
                          "Failed to create semaphores.");
  }
}

void Init(uint32_t w, uint32_t h) {
  SwapChainSupportDetails swapChainSupport =
      QuerySwapChainSupport(g_physicalDevice, g_surface);

  vk::SurfaceFormatKHR format =
      swapChainSupport.formats[0]; // safe since it has been checked that
                                   // there are available formats
  if (swapChainSupport.formats.size() == 1 &&
      swapChainSupport.formats[0].format == vk::Format::eUndefined)
    format = PreferredImageFormat;
  else {
    bool preferredFormatFound = false;
    for (const auto &availableFormat : swapChainSupport.formats) {
      if (availableFormat == PreferredImageFormat) {
        format = availableFormat;
        preferredFormatFound = true;
      }
    }

    if (!preferredFormatFound)
      printf(
          "[WARNING] Specified preferred swapchain image format {%s, %s} not "
          "supported. Using {%s, %s} instead.\n",
          vk::to_string(PreferredImageFormat.format).c_str(),
          vk::to_string(PreferredImageFormat.colorSpace).c_str(),
          vk::to_string(format.format).c_str(),
          vk::to_string(format.colorSpace).c_str());
  }

  vk::PresentModeKHR presentMode =
      vk::PresentModeKHR::eFifo; // Fifo mode's availability is guaranteed
  if (PreferredPresentMode != vk::PresentModeKHR::eFifo) {
    for (const auto &availablePresentMode : swapChainSupport.presentModes) {
      if (availablePresentMode == PreferredPresentMode) {
        presentMode = availablePresentMode;
        break;
      }
    }

    if (presentMode == vk::PresentModeKHR::eFifo) {
      PreferredPresentMode = vk::PresentModeKHR::eFifo;
      std::cout << vk::to_string(PreferredPresentMode)
                << " not supported. Using FIFO V-Sync mode instead."
                << std::endl;
    }
  }

  if (swapChainSupport.capabilities.currentExtent.width !=
      std::numeric_limits<uint32_t>::max()) {
    extent = swapChainSupport.capabilities.currentExtent;
  } else {
    VkExtent2D actualExtent = {w, h};

    actualExtent.width =
        std::max(swapChainSupport.capabilities.minImageExtent.width,
                 std::min(swapChainSupport.capabilities.maxImageExtent.width,
                          actualExtent.width));
    actualExtent.height =
        std::max(swapChainSupport.capabilities.minImageExtent.height,
                 std::min(swapChainSupport.capabilities.maxImageExtent.height,
                          actualExtent.height));

    extent = actualExtent;
  }

  uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
  if (swapChainSupport.capabilities.maxImageCount > 0 &&
      imageCount > swapChainSupport.capabilities.maxImageCount)
    imageCount = swapChainSupport.capabilities.maxImageCount;

  QueueFamilyIndices indices = GetQueueFamilies(g_physicalDevice, g_surface);
  uint32_t queueFamilyIndices[] = {(uint32_t)indices.graphicsFamily,
                                   (uint32_t)indices.presentFamily};

  vk::SharingMode imageSharingMode = vk::SharingMode::eExclusive;
  uint32_t queueFamilyIndexCount = 0;

  if (indices.graphicsFamily != indices.presentFamily) {
    imageSharingMode = vk::SharingMode::eConcurrent;
    queueFamilyIndexCount = sizeof(queueFamilyIndices) / sizeof(uint32_t);
  }

  vk::SwapchainCreateInfoKHR createInfo(
      vk::SwapchainCreateFlagsKHR(), g_surface, imageCount, format.format,
      format.colorSpace, extent, 1, vk::ImageUsageFlagBits::eColorAttachment,
      imageSharingMode, queueFamilyIndexCount, queueFamilyIndices,
      swapChainSupport.capabilities.currentTransform,
      vk::CompositeAlphaFlagBitsKHR::eOpaque, presentMode, VK_TRUE
      // swapchain
  );

  g_device.createSwapchainKHR(&createInfo, g_allocationCallbacks, &handle);

  images = g_device.getSwapchainImagesKHR(handle);
  imageFormat = format.format;

  vk::ImageSubresourceRange imageSubresourceRange(
      vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);

  imageViews.resize(images.size());
  for (size_t i = 0; i < images.size(); i++) {
    vk::ImageViewCreateInfo createInfo(
        vk::ImageViewCreateFlags(), images[i], vk::ImageViewType::e2D,
        imageFormat, vk::ComponentMapping(), imageSubresourceRange);

    CHECK_VK_RESULT_FATAL(g_device.createImageView(&createInfo,
                                                   g_allocationCallbacks,
                                                   &imageViews[i]),
                          "Failed to create image views");
  }

  InitSyncBarriers();

// TODO Move in some debug tool
#ifndef NDEBUG
  g_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eSwapchainKHR,
                                       (uint64_t)((VkSwapchainKHR)handle),
                                       "Swapchain"},
                                      g_dldy);

  g_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eQueue,
                                       (uint64_t)((VkQueue)presentQueue.handle),
                                       "Present Queue"},
                                      g_dldy);
#endif
}

void Destroy() {
  for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++) {
    g_device.destroySemaphore(imageAvailableSemaphores[i],
                              g_allocationCallbacks);

    g_device.destroyFence(inFlightFences[i], g_allocationCallbacks);
  }

  for (size_t i = 0; i < imageViews.size(); ++i) {
    g_device.destroyImageView(imageViews[i], g_allocationCallbacks);
  }

  g_device.destroySwapchainKHR(handle, g_allocationCallbacks);
}

vk::Result AcquireNextImage() {
  g_device.waitForFences(1, &inFlightFences[currentFrame], VK_TRUE,
                         std::numeric_limits<uint64_t>::max());

  g_device.resetFences(1, &inFlightFences[currentFrame]);

  vk::Result result = g_device.acquireNextImageKHR(
      handle, std::numeric_limits<uint64_t>::max(),
      imageAvailableSemaphores[currentFrame], nullptr, &currentImageIndex);

  return result;
}

vk::Result Present(const vk::Semaphore *presentSemaphore) {
  vk::PresentInfoKHR presentInfo(1, presentSemaphore, 1, &handle,
                                 &currentImageIndex, nullptr);

  currentFrame = (currentFrame + 1) % MAX_IN_FLIGHT_FRAMES;

  return presentQueue.handle.presentKHR(&presentInfo);
}

} // namespace Swapchain
} // namespace Renderer
