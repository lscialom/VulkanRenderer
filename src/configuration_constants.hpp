#include "debug_tools.hpp"

//-----------------------------------------------------------------------------
// MAX VALUES
//-----------------------------------------------------------------------------

static constexpr const uint8_t MAX_IN_FLIGHT_FRAMES = 2;
static constexpr const uint8_t MAX_OBJECT_INSTANCES_PER_TEMPLATE = 100;

//-----------------------------------------------------------------------------
// INDICES TYPE
//-----------------------------------------------------------------------------

#define VERTEX_INDICES_TYPE uint16_t
#define VULKAN_INDICES_TYPE                                                    \
  (sizeof(VERTEX_INDICES_TYPE) == sizeof(uint16_t) ? vk::IndexType::eUint16    \
                                                   : vk::IndexType::eUint32)

//-----------------------------------------------------------------------------
// EXTENSIONS
//-----------------------------------------------------------------------------

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
