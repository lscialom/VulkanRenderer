#pragma once

#include "debug_tools.hpp"

#include <fstream>
#include <set>

#include <constants.glsl>

//-----------------------------------------------------------------------------
// MAX VALUES
//-----------------------------------------------------------------------------

static constexpr const uint8_t MAX_IN_FLIGHT_FRAMES = 2;
static constexpr const size_t MAX_OBJECT_INSTANCES_PER_TEMPLATE = 100;

//-----------------------------------------------------------------------------
// INDICES TYPE
//-----------------------------------------------------------------------------

#define VERTEX_INDICES_TYPE uint32_t
#define VULKAN_INDICES_TYPE                                                    \
  (sizeof(VERTEX_INDICES_TYPE) == sizeof(uint16_t) ? vk::IndexType::eUint16    \
                                                   : vk::IndexType::eUint32)

//-----------------------------------------------------------------------------
// EXTENSIONS
//-----------------------------------------------------------------------------

static constexpr const std::array<const char *, 2>
    g_requiredInstanceExtensions = {
        VK_KHR_SURFACE_EXTENSION_NAME,
#ifdef _WIN32
        VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
#endif
};

static constexpr const std::array<const char *, 1> g_deviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME};

#ifndef NDEBUG
static constexpr const uint8_t INSTANCE_DEBUG_EXTENSION_COUNT = 1;
#else
static constexpr const uint8_t INSTANCE_DEBUG_EXTENSION_COUNT = 0;
#endif

static constexpr const uint8_t INSTANCE_EXTENSION_COUNT =
    0 + INSTANCE_DEBUG_EXTENSION_COUNT;
static constexpr const std::array<const char *, INSTANCE_EXTENSION_COUNT>
    g_instanceExtensions = {
#ifndef NDEBUG
        VK_EXT_DEBUG_UTILS_EXTENSION_NAME
#endif
};

//-----------------------------------------------------------------------------
// DEBUG
//-----------------------------------------------------------------------------

#ifndef NDEBUG
static constexpr const std::array<const char *, 2> g_validationLayers = {
    "VK_LAYER_LUNARG_standard_validation", "VK_LAYER_LUNARG_monitor"};

#define DEBUG_CALLBACK_RETURN_TYPE VkBool32

#define DEBUG_CALLBACK_ARGUMENTS                                               \
  VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,                      \
      VkDebugUtilsMessageTypeFlagsEXT messageType,                             \
      const VkDebugUtilsMessengerCallbackDataEXT *callbackData, void *userData

struct DebugMessengerInfo {
  using DebugCallbackfn = DEBUG_CALLBACK_RETURN_TYPE(VKAPI_CALL *)(
      DEBUG_CALLBACK_ARGUMENTS) VKAPI_ATTR;

  const char *name;

  vk::DebugUtilsMessageSeverityFlagsEXT severityFlags;
  vk::DebugUtilsMessageTypeFlagsEXT typeFlags;

  DebugCallbackfn callback;
  void *userData = nullptr;

  vk::DebugUtilsMessengerCreateInfoEXT MakeCreateInfo() const {
    return vk::DebugUtilsMessengerCreateInfoEXT(
        vk::DebugUtilsMessengerCreateFlagsEXT(), severityFlags, typeFlags,
        callback, userData);
  }
};

static const std::array<const DebugMessengerInfo, 1> g_debugMessengersInfos{
    {{"Debug Messenger",
      vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose |
          vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning |
          vk::DebugUtilsMessageSeverityFlagBitsEXT::eError,
      vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral |
          vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation |
          vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance,
      [](DEBUG_CALLBACK_ARGUMENTS) -> DEBUG_CALLBACK_RETURN_TYPE {
        std::cout << GetFullFormattedDebugMessage(messageSeverity, messageType,
                                                  callbackData)
                  << std::endl;
        return VK_FALSE;
      },
      nullptr}}};

#undef DEBUG_CALLBACK_ARGUMENTS
#undef DEBUG_CALLBACK_RETURN_TYPE

#endif

//-----------------------------------------------------------------------------
// CONFIGURATION HELPERS
//-----------------------------------------------------------------------------

struct QueueFamilyIndices {
  int graphicsFamily = -1;
  int presentFamily = -1;
  int transferFamily = -1;

  bool isComplete() {
    return graphicsFamily >= 0 && presentFamily >= 0 && transferFamily >= 0;
  }
};

static QueueFamilyIndices GetQueueFamilies(vk::PhysicalDevice device,
                                           vk::SurfaceKHR surface) {
  std::vector<vk::QueueFamilyProperties> queueFamilies =
      device.getQueueFamilyProperties();
  QueueFamilyIndices indices;

  int i = 0;
  for (const auto &queueFamily : queueFamilies) {
    if (queueFamily.queueCount > 0) {
      if (queueFamily.queueFlags & vk::QueueFlagBits::eGraphics)
        indices.graphicsFamily = i;

      if (device.getSurfaceSupportKHR(i, surface))
        indices.presentFamily = i;

      if (queueFamily.queueFlags & vk::QueueFlagBits::eTransfer)
        indices.transferFamily = i;
    }

    if (indices.isComplete())
      break;

    i++;
  }

  return indices;
}

struct SwapChainSupportDetails {
  vk::SurfaceCapabilitiesKHR capabilities;
  std::vector<vk::SurfaceFormatKHR> formats;
  std::vector<vk::PresentModeKHR> presentModes;
};

static SwapChainSupportDetails QuerySwapChainSupport(vk::PhysicalDevice device,
                                                     vk::SurfaceKHR surface) {
  SwapChainSupportDetails details;

  details.capabilities = device.getSurfaceCapabilitiesKHR(surface);
  details.formats = device.getSurfaceFormatsKHR(surface);
  details.presentModes = device.getSurfacePresentModesKHR(surface);

  return details;
}

static vk::Format FindSupportedFormat(vk::PhysicalDevice device,
                                      const std::vector<vk::Format> &candidates,
                                      vk::ImageTiling tiling,
                                      vk::FormatFeatureFlags features) {
  for (vk::Format format : candidates) {
    vk::FormatProperties props = device.getFormatProperties(format);

    if (tiling == vk::ImageTiling::eLinear &&
        (props.linearTilingFeatures & features) == features) {
      return format;
    } else if (tiling == vk::ImageTiling::eOptimal &&
               (props.optimalTilingFeatures & features) == features) {
      return format;
    }
  }

  throw std::runtime_error("failed to find supported format!");
}

static bool CheckDefaultDeviceExtensionsSupport(vk::PhysicalDevice device) {
  const std::vector<vk::ExtensionProperties> availableExtensions =
      device.enumerateDeviceExtensionProperties();

  std::set<std::string> requiredExtensions(g_deviceExtensions.begin(),
                                           g_deviceExtensions.end());

  for (const auto &extension : availableExtensions)
    requiredExtensions.erase(extension.extensionName);

  return requiredExtensions.empty();
}

template <typename Iter>
static bool CheckDeviceExtensionSupport(vk::PhysicalDevice device, Iter begin,
                                        Iter end) {
  const std::vector<vk::ExtensionProperties> availableExtensions =
      device.enumerateDeviceExtensionProperties();

  std::set<std::string> requiredExtensions(begin, end);

  for (const auto &extension : availableExtensions)
    requiredExtensions.erase(extension.extensionName);

  return requiredExtensions.empty();
}

static uint64_t RateDeviceSuitability(vk::PhysicalDevice device,
                                      vk::SurfaceKHR surface) {
  if (!GetQueueFamilies(device, surface).isComplete() ||
      !CheckDefaultDeviceExtensionsSupport(device))
    return 0;

  // Done separately since we need to check for the swapchain extension support
  // first
  SwapChainSupportDetails swapChainSupport =
      QuerySwapChainSupport(device, surface);
  if (swapChainSupport.formats.empty() || swapChainSupport.presentModes.empty())
    return 0;

  vk::PhysicalDeviceProperties deviceProperties = device.getProperties();
  // vk::PhysicalDeviceFeatures deviceFeatures = device.getFeatures();

  uint64_t score = 0;

  // Discrete GPUs have a significant performance advantage
  if (deviceProperties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu)
    score += 1000;

  // Maximum possible size of textures affects graphics quality
  score += deviceProperties.limits.maxImageDimension2D;

  vk::PhysicalDeviceMemoryProperties memProperties =
      device.getMemoryProperties();

  for (size_t i = 0; i < memProperties.memoryHeapCount; ++i)
    score += memProperties.memoryHeaps[i].size;

  return score;
}
