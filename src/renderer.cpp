#include "renderer.hpp"
#include "window_handler.hpp"

#include <vulkan/vulkan.hpp>

#include <Eigen/Dense>

#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"

#include <fstream>
#include <iostream>

#include <map>
#include <set>
#include <unordered_map>
#include <vector>

//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------

// Redundant pattern
#define CHECK_VK_RESULT_FATAL(vk_function, msg)                                \
  {                                                                            \
    vk::Result res;                                                            \
    if ((res = (vk::Result)vk_function) != vk::Result::eSuccess) {             \
      std::string err = "[FATAL]";                                             \
      err += msg;                                                              \
      err += " : ";                                                            \
      err += vk::to_string(res);                                               \
      throw std::runtime_error(err.c_str());                                   \
    }                                                                          \
  }

// For visibility's sake
#define TRY_CATCH_BLOCK(msg, code)                                             \
  try {                                                                        \
    code                                                                       \
  } catch (const std::exception &e) {                                          \
    std::cerr << e.what() << "\n"                                              \
              << "[FATAL]" << msg << std::endl;                                \
    abort();                                                                   \
  }

namespace Renderer {
//-----------------------------------------------------------------------------
// FORWARD DEFINITIONS
//-----------------------------------------------------------------------------

#ifndef NDEBUG
static std::string GetFullFormattedDebugMessage(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData);
#endif

//-----------------------------------------------------------------------------
// CONFIGURATION CONSTANTS
//-----------------------------------------------------------------------------

static constexpr const uint8_t MAX_IN_FLIGHT_FRAMES = 2;

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

#define VERTEX_INDICES_TYPE uint16_t
#define VULKAN_INDICES_TYPE                                                    \
  (sizeof(VERTEX_INDICES_TYPE) == sizeof(uint16_t) ? vk::IndexType::eUint16    \
                                                   : vk::IndexType::eUint32)

//-----------------------------------------------------------------------------
// GLOBAL CONTEXT
//-----------------------------------------------------------------------------

static vk::Instance g_instance;
static vk::PhysicalDevice g_physicalDevice;

static vk::Device g_device;

static vk::SurfaceKHR g_surface;

static vk::Format g_requiredFormat;
static vk::Extent2D g_extent;

struct Queue {
  int index = -1;
  vk::Queue handle = nullptr;
};

static Queue g_graphicsQueue;
static Queue g_presentQueue;

static vk::CommandPool g_commandPool;
static vk::CommandPool g_stagingCommandPool;

static vk::RenderPass g_renderPass;

static vk::AllocationCallbacks *g_allocator = nullptr;

static vk::DispatchLoaderDynamic g_dldy;

static bool g_framebufferResized = false;

//-----------------------------------------------------------------------------
// CONFIGURATION HELPERS
//-----------------------------------------------------------------------------

struct QueueFamilyIndices {
  int graphicsFamily = -1;
  int presentFamily = -1;

  bool isComplete() { return graphicsFamily >= 0 && presentFamily >= 0; }
};

static QueueFamilyIndices GetQueueFamilies(vk::PhysicalDevice device) {
  std::vector<vk::QueueFamilyProperties> queueFamilies =
      device.getQueueFamilyProperties();
  QueueFamilyIndices indices;

  int i = 0;
  for (const auto &queueFamily : queueFamilies) {
    if (queueFamily.queueCount > 0) {
      if (queueFamily.queueFlags & vk::QueueFlagBits::eGraphics)
        indices.graphicsFamily = i;

      if (device.getSurfaceSupportKHR(i, g_surface))
        indices.presentFamily = i;
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

static SwapChainSupportDetails
QuerySwapChainSupport(vk::PhysicalDevice device) {
  SwapChainSupportDetails details;

  details.capabilities = device.getSurfaceCapabilitiesKHR(g_surface);
  details.formats = device.getSurfaceFormatsKHR(g_surface);
  details.presentModes = device.getSurfacePresentModesKHR(g_surface);

  return details;
}

static bool CheckDeviceExtensionSupport(vk::PhysicalDevice device) {
  const std::vector<vk::ExtensionProperties> availableExtensions =
      device.enumerateDeviceExtensionProperties();

  std::set<std::string> requiredExtensions(g_deviceExtensions.begin(),
                                           g_deviceExtensions.end());

  for (const auto &extension : availableExtensions)
    requiredExtensions.erase(extension.extensionName);

  return requiredExtensions.empty();
}

static int RateDeviceSuitability(vk::PhysicalDevice device) {
  if (!GetQueueFamilies(device).isComplete() ||
      !CheckDeviceExtensionSupport(device))
    return 0;

  // Done separately since we need to check for the swapchain extension support
  // first
  SwapChainSupportDetails swapChainSupport = QuerySwapChainSupport(device);
  if (swapChainSupport.formats.empty() || swapChainSupport.presentModes.empty())
    return 0;

  vk::PhysicalDeviceProperties deviceProperties = device.getProperties();
  // vk::PhysicalDeviceFeatures deviceFeatures = device.getFeatures();

  int score = 0;

  // Discrete GPUs have a significant performance advantage
  if (deviceProperties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu)
    score += 1000;

  // Maximum possible size of textures affects graphics quality
  score += deviceProperties.limits.maxImageDimension2D;

  return score;
}

static vk::ShaderModule CreateShaderModule(const std::vector<char> &code) {
  vk::ShaderModuleCreateInfo createInfo(
      vk::ShaderModuleCreateFlags(), code.size(),
      reinterpret_cast<const uint32_t *>(code.data()));

  vk::ShaderModule shaderModule;
  CHECK_VK_RESULT_FATAL(
      g_device.createShaderModule(&createInfo, g_allocator, &shaderModule),
      "Failed to create shader module");

  return shaderModule;
}

//-----------------------------------------------------------------------------
// FILE UTILS
//-----------------------------------------------------------------------------

static std::vector<char> ReadFile(const std::string &filename) {
  std::ifstream file(filename, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "[ERROR] "
              << "Could not open file " << filename << std::endl;
    return {};
  }

  size_t fileSize = (size_t)file.tellg();
  std::vector<char> buffer(fileSize);

  file.seekg(0);
  file.read(buffer.data(), fileSize);

  file.close();

  return buffer;
}

//-----------------------------------------------------------------------------
// RENDER DATA STRUCTURES
//-----------------------------------------------------------------------------

struct Vertex {
  Eigen::Vector2f pos;
  Eigen::Vector3f color;

  static vk::VertexInputBindingDescription get_binding_description() {
    return vk::VertexInputBindingDescription(0, sizeof(Vertex),
                                             vk::VertexInputRate::eVertex);
  }

  static std::array<vk::VertexInputAttributeDescription, 2>
  get_attribute_descriptions() {
    return std::array<vk::VertexInputAttributeDescription, 2>{
        {{0, 0, vk::Format::eR32G32Sfloat, offsetof(Vertex, pos)},
         {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(Vertex, color)}}};
  }
};

struct UniformBufferObject {
  Eigen::Matrix4f model;
  Eigen::Matrix4f view;
  Eigen::Matrix4f proj;
};

static const std::vector<Vertex> g_vertices = {
    {{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
    {{0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}},
    {{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
    {{-0.5f, 0.5f}, {1.0f, 1.0f, 1.0f}}};

static const std::vector<VERTEX_INDICES_TYPE> g_indices = {0, 1, 2, 2, 3, 0};

//-----------------------------------------------------------------------------
// MEMORY
//-----------------------------------------------------------------------------

static VmaAllocator g_vmaAllocator;

struct Buffer {
  vk::Buffer handle;
  VmaAllocation allocation;

  Buffer() = default;

  Buffer(const Buffer &other) = delete;
  Buffer(Buffer &&other) {
    handle = other.handle;
    other.handle = nullptr;

    allocation = std::move(other.allocation);
  }

  ~Buffer() {
    if (handle)
      vmaDestroyBuffer(g_vmaAllocator, handle, allocation);
  }

  Buffer &operator=(const Buffer &o) = delete;
  Buffer &operator=(Buffer &&other) {
    handle = other.handle;
    other.handle = nullptr;

    allocation = std::move(other.allocation);

    return *this;
  }
};

static void CreateBuffer(VkDeviceSize size, vk::BufferUsageFlags usage,
                         vk::MemoryPropertyFlags properties, vk::Buffer *buffer,
                         VmaAllocation *allocation) {
  VkBufferCreateInfo bufferInfo = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  bufferInfo.size = size;
  bufferInfo.usage = (VkBufferUsageFlags)usage;
  bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocInfo = {};
  allocInfo.usage = VMA_MEMORY_USAGE_UNKNOWN;
  allocInfo.requiredFlags = (VkMemoryPropertyFlags)properties;

  CHECK_VK_RESULT_FATAL(vmaCreateBuffer(g_vmaAllocator, &bufferInfo, &allocInfo,
                                        (VkBuffer *)buffer, allocation,
                                        nullptr),
                        "Failed to create vertex buffer.");
}

static void CopyBuffer(vk::Buffer srcBuffer, vk::Buffer dstBuffer,
                       VkDeviceSize size) {
  vk::CommandBufferAllocateInfo allocInfo(g_stagingCommandPool,
                                          vk::CommandBufferLevel::ePrimary, 1);

  vk::CommandBuffer commandBuffer;
  g_device.allocateCommandBuffers(&allocInfo, &commandBuffer);

  vk::CommandBufferBeginInfo beginInfo(
      vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  commandBuffer.begin(&beginInfo);

  vk::BufferCopy copyRegion(0, // TODO src offset
                            0, // TODO dst offset
                            size);

  commandBuffer.copyBuffer(srcBuffer, dstBuffer, 1, &copyRegion);

  commandBuffer.end();

  vk::SubmitInfo submitInfo = {};
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandBuffer;

  g_graphicsQueue.handle.submit(1, &submitInfo, nullptr);
  g_graphicsQueue.handle.waitIdle();

  g_device.freeCommandBuffers(g_stagingCommandPool, 1, &commandBuffer);
}

static void CreateVertexBuffer(const std::vector<Vertex> &vertexBuffer,
                               vk::Buffer &buffer, VmaAllocation &allocation) {
  VkDeviceSize bufferSize = sizeof(Vertex) * vertexBuffer.size();

  Buffer stagingBuffer;

  CreateBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
               vk::MemoryPropertyFlagBits::eHostVisible |
                   vk::MemoryPropertyFlagBits::eHostCoherent,
               &stagingBuffer.handle, &stagingBuffer.allocation);

  void *data;
  vmaMapMemory(g_vmaAllocator, stagingBuffer.allocation, &data);
  memcpy(data, vertexBuffer.data(), (size_t)bufferSize);
  vmaUnmapMemory(g_vmaAllocator, stagingBuffer.allocation);

  CreateBuffer(bufferSize,
               vk::BufferUsageFlagBits::eTransferDst |
                   vk::BufferUsageFlagBits::eVertexBuffer,
               vk::MemoryPropertyFlagBits::eDeviceLocal, &buffer, &allocation);

  CopyBuffer(stagingBuffer.handle, buffer, bufferSize);
}

static void
CreateIndexBuffer(const std::vector<VERTEX_INDICES_TYPE> &indexBuffer,
                  vk::Buffer &buffer, VmaAllocation &allocation) {
  VkDeviceSize bufferSize = sizeof(VERTEX_INDICES_TYPE) * indexBuffer.size();

  Buffer stagingBuffer;

  CreateBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
               vk::MemoryPropertyFlagBits::eHostVisible |
                   vk::MemoryPropertyFlagBits::eHostCoherent,
               &stagingBuffer.handle, &stagingBuffer.allocation);

  void *data;
  vmaMapMemory(g_vmaAllocator, stagingBuffer.allocation, &data);
  memcpy(data, indexBuffer.data(), (size_t)bufferSize);
  vmaUnmapMemory(g_vmaAllocator, stagingBuffer.allocation);

  CreateBuffer(bufferSize,
               vk::BufferUsageFlagBits::eTransferDst |
                   vk::BufferUsageFlagBits::eIndexBuffer,
               vk::MemoryPropertyFlagBits::eDeviceLocal, &buffer, &allocation);

  CopyBuffer(stagingBuffer.handle, buffer, bufferSize);
}

static vk::DeviceSize
CreateVIBuffer(const std::vector<Vertex> &vertexBuffer,
               const std::vector<VERTEX_INDICES_TYPE> &indexBuffer,
               vk::Buffer &buffer, VmaAllocation &allocation) {
  VkDeviceSize iBufferSize = sizeof(VERTEX_INDICES_TYPE) * indexBuffer.size();
  VkDeviceSize vBufferSize = sizeof(Vertex) * vertexBuffer.size();

  VkDeviceSize bufferSize = iBufferSize + vBufferSize;
  VkDeviceSize offset = iBufferSize;

  Buffer stagingBuffer;
  CreateBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
               vk::MemoryPropertyFlagBits::eHostVisible |
                   vk::MemoryPropertyFlagBits::eHostCoherent,
               &stagingBuffer.handle, &stagingBuffer.allocation);

  void *data;
  vmaMapMemory(g_vmaAllocator, stagingBuffer.allocation, &data);
  memcpy(data, indexBuffer.data(), (size_t)iBufferSize);
  memcpy((char *)data + offset, vertexBuffer.data(), (size_t)vBufferSize);
  vmaUnmapMemory(g_vmaAllocator, stagingBuffer.allocation);

  CreateBuffer(bufferSize,
               vk::BufferUsageFlagBits::eTransferDst |
                   vk::BufferUsageFlagBits::eVertexBuffer |
                   vk::BufferUsageFlagBits::eIndexBuffer,
               vk::MemoryPropertyFlagBits::eDeviceLocal, &buffer, &allocation);

  CopyBuffer(stagingBuffer.handle, buffer, bufferSize);

  return offset;
}

//-----------------------------------------------------------------------------
// RENDER CONTEXT
//-----------------------------------------------------------------------------

struct Shader {
private:
  void init_pipeline(const std::string &vertPath, const std::string &fragPath) {
    auto vertShaderCode = ReadFile(vertPath);
    auto fragShaderCode = ReadFile(fragPath);

    vk::ShaderModule vertShaderModule = CreateShaderModule(vertShaderCode);
    vk::ShaderModule fragShaderModule = CreateShaderModule(fragShaderCode);

    vk::PipelineShaderStageCreateInfo vertShaderStageInfo(
        vk::PipelineShaderStageCreateFlags(), vk::ShaderStageFlagBits::eVertex,
        vertShaderModule, "main");

    vk::PipelineShaderStageCreateInfo fragShaderStageInfo(
        vk::PipelineShaderStageCreateFlags(),
        vk::ShaderStageFlagBits::eFragment, fragShaderModule, "main");

    vk::PipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo,
                                                        fragShaderStageInfo};

    auto bindingDescription = Vertex::get_binding_description();
    auto attributeDescriptions = Vertex::get_attribute_descriptions();

    vk::PipelineVertexInputStateCreateInfo vertexInputInfo(
        vk::PipelineVertexInputStateCreateFlags(), 1, &bindingDescription,
        static_cast<uint32_t>(attributeDescriptions.size()),
        attributeDescriptions.data());

    vk::PipelineInputAssemblyStateCreateInfo inputAssembly(
        vk::PipelineInputAssemblyStateCreateFlags(),
        vk::PrimitiveTopology::eTriangleList, VK_FALSE);

    vk::Viewport viewport(0, 0, (float)g_extent.width, (float)g_extent.height,
                          0, 1);

    vk::Rect2D scissor({0, 0}, g_extent);

    vk::PipelineViewportStateCreateInfo viewportState(
        vk::PipelineViewportStateCreateFlags(), 1, &viewport, 1, &scissor);

    vk::PipelineRasterizationStateCreateInfo rasterizer(
        vk::PipelineRasterizationStateCreateFlags(), VK_FALSE, VK_FALSE,
        vk::PolygonMode::eFill, vk::CullModeFlagBits::eBack,
        vk::FrontFace::eClockwise, VK_FALSE, 0, 0, 0, 1);

    vk::PipelineMultisampleStateCreateInfo multisampling(
        vk::PipelineMultisampleStateCreateFlags(), vk::SampleCountFlagBits::e1,
        VK_FALSE, 1, nullptr, VK_FALSE, VK_FALSE);

    vk::PipelineColorBlendAttachmentState colorBlendAttachment(
        VK_TRUE, vk::BlendFactor::eSrcAlpha, vk::BlendFactor::eOneMinusSrcAlpha,
        vk::BlendOp::eAdd, vk::BlendFactor::eOne, vk::BlendFactor::eZero,
        vk::BlendOp::eAdd,
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

    vk::PipelineColorBlendStateCreateInfo colorBlending(
        vk::PipelineColorBlendStateCreateFlags(), VK_FALSE, vk::LogicOp::eCopy,
        1, &colorBlendAttachment);

    vk::DynamicState dynamicState = vk::DynamicState::eViewport;
    vk::PipelineDynamicStateCreateInfo dynamicStateCreateInfo(
        vk::PipelineDynamicStateCreateFlags(), 1, &dynamicState);

    vk::PipelineLayoutCreateInfo pipelineLayoutInfo(
        vk::PipelineLayoutCreateFlags(), 0, nullptr, 0, nullptr);

    CHECK_VK_RESULT_FATAL(g_device.createPipelineLayout(&pipelineLayoutInfo,
                                                        g_allocator,
                                                        &pipelineLayout),
                          "Failed to create pipeline layout.");

    vk::GraphicsPipelineCreateInfo pipelineInfo(
        vk::PipelineCreateFlags(), 2, shaderStages, &vertexInputInfo,
        &inputAssembly, nullptr, &viewportState, &rasterizer, &multisampling,
        nullptr, &colorBlending, &dynamicStateCreateInfo, pipelineLayout,
        g_renderPass, 0, nullptr, -1);

    CHECK_VK_RESULT_FATAL(
        g_device.createGraphicsPipelines(nullptr, 1, &pipelineInfo, g_allocator,
                                         &pipeline),
        "Failed to create pipeline layout.");

    g_device.destroyShaderModule(fragShaderModule, g_allocator);
    g_device.destroyShaderModule(vertShaderModule, g_allocator);
  }

public:
  vk::Pipeline pipeline;
  vk::PipelineLayout pipelineLayout;

  void init(const std::string &vertPath, const std::string &fragPath) {
    init_pipeline(vertPath, fragPath);
  }

  void destroy() {
    g_device.destroyPipeline(pipeline, g_allocator);
    g_device.destroyPipelineLayout(pipelineLayout, g_allocator);
  }
};

Shader g_baseShader;

struct Object {
  Buffer buffer;
  vk::DeviceSize offset = 0;

  uint64_t nbIndices = 0;

  void init(const std::vector<Vertex> &vertices,
            const std::vector<VERTEX_INDICES_TYPE> &indices) {
    offset =
        CreateVIBuffer(vertices, indices, buffer.handle, buffer.allocation);

    nbIndices = indices.size();
  }

  void record(const vk::CommandBuffer &commandbuffer) const {
    commandbuffer.bindIndexBuffer(buffer.handle, 0, VULKAN_INDICES_TYPE);
    commandbuffer.bindVertexBuffers(0, 1, &buffer.handle, &offset);
    commandbuffer.drawIndexed(nbIndices, 1, 0, 0, 0);
  }
};

static struct {
  vk::SwapchainKHR swapchain;

  std::vector<vk::Image> images;
  std::vector<vk::ImageView> imageViews;

  std::vector<vk::Framebuffer> framebuffers;
  std::vector<vk::CommandBuffer> commandbuffers;

  vk::PresentModeKHR requiredPresentMode = vk::PresentModeKHR::eMailbox;

  std::unordered_map<Shader *, std::vector<Object>> objects;

  void init_swapchain() {
    SwapChainSupportDetails swapChainSupport =
        QuerySwapChainSupport(g_physicalDevice);

    vk::SurfaceFormatKHR format =
        swapChainSupport.formats[0]; // safe since it has been checked that
                                     // there are available formats
    if (swapChainSupport.formats.size() == 1 &&
        swapChainSupport.formats[0].format == vk::Format::eUndefined)
      format = {vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear};
    else {
      for (const auto &availableFormat : swapChainSupport.formats) {
        if (availableFormat.format == vk::Format::eB8G8R8A8Unorm &&
            availableFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear)
          format = availableFormat;
      }
    }

    vk::PresentModeKHR presentMode =
        vk::PresentModeKHR::eFifo; // Fifo mode's availability is guaranteed
    if (requiredPresentMode != vk::PresentModeKHR::eFifo) {
      for (const auto &availablePresentMode : swapChainSupport.presentModes) {
        if (availablePresentMode == requiredPresentMode) {
          presentMode = availablePresentMode;
          break;
        }
      }

      if (presentMode == vk::PresentModeKHR::eFifo) {
        requiredPresentMode = vk::PresentModeKHR::eFifo;
        std::cout << vk::to_string(requiredPresentMode)
                  << " not supported. Using FIFO V-Sync mode instead."
                  << std::endl;
      }
    }

    if (swapChainSupport.capabilities.currentExtent.width !=
        std::numeric_limits<uint32_t>::max()) {
      g_extent = swapChainSupport.capabilities.currentExtent;
    } else {
      int width, height;
      WindowHandler::GetFramebufferSize(&width, &height);

      VkExtent2D actualExtent = {width, height};

      actualExtent.width =
          std::max(swapChainSupport.capabilities.minImageExtent.width,
                   std::min(swapChainSupport.capabilities.maxImageExtent.width,
                            actualExtent.width));
      actualExtent.height =
          std::max(swapChainSupport.capabilities.minImageExtent.height,
                   std::min(swapChainSupport.capabilities.maxImageExtent.height,
                            actualExtent.height));

      g_extent = actualExtent;
    }

    uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
    if (swapChainSupport.capabilities.maxImageCount > 0 &&
        imageCount > swapChainSupport.capabilities.maxImageCount)
      imageCount = swapChainSupport.capabilities.maxImageCount;

    QueueFamilyIndices indices = GetQueueFamilies(g_physicalDevice);
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
        format.colorSpace, g_extent, 1,
        vk::ImageUsageFlagBits::eColorAttachment, imageSharingMode,
        queueFamilyIndexCount, queueFamilyIndices,
        swapChainSupport.capabilities.currentTransform,
        vk::CompositeAlphaFlagBitsKHR::eOpaque, presentMode, VK_TRUE
        // swapchain
    );

    g_device.createSwapchainKHR(&createInfo, g_allocator, &swapchain);

    images = g_device.getSwapchainImagesKHR(swapchain);
    g_requiredFormat = format.format;

    vk::ImageSubresourceRange imageSubresourceRange(
        vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);

    imageViews.resize(images.size());
    for (size_t i = 0; i < images.size(); i++) {
      vk::ImageViewCreateInfo createInfo(
          vk::ImageViewCreateFlags(), images[i], vk::ImageViewType::e2D,
          g_requiredFormat, vk::ComponentMapping(), imageSubresourceRange);

      CHECK_VK_RESULT_FATAL(
          g_device.createImageView(&createInfo, g_allocator, &imageViews[i]),
          "Failed to create image views");
    }
  }

  void destroy_swapchain() {
    for (auto imageView : imageViews)
      g_device.destroyImageView(imageView, g_allocator);

    g_device.destroySwapchainKHR(swapchain, g_allocator);
  }

  void init_render_pass() {
    vk::AttachmentDescription colorAttachment(
        vk::AttachmentDescriptionFlags(), g_requiredFormat,
        vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
        vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
        vk::AttachmentStoreOp::eDontCare, vk::ImageLayout::eUndefined,
        vk::ImageLayout::ePresentSrcKHR);

    vk::AttachmentReference colorAttachmentRef(
        0, vk::ImageLayout::eColorAttachmentOptimal);

    vk::SubpassDescription subpass(vk::SubpassDescriptionFlags(),
                                   vk::PipelineBindPoint::eGraphics, 0, nullptr,
                                   1, &colorAttachmentRef);

    vk::SubpassDependency dependency(
        VK_SUBPASS_EXTERNAL, 0,
        vk::PipelineStageFlagBits::eColorAttachmentOutput,
        vk::PipelineStageFlagBits::eColorAttachmentOutput,
        (vk::AccessFlagBits)0,
        vk::AccessFlagBits::eColorAttachmentRead |
            vk::AccessFlagBits::eColorAttachmentWrite);

    vk::RenderPassCreateInfo renderPassInfo(vk::RenderPassCreateFlags(), 1,
                                            &colorAttachment, 1, &subpass, 1,
                                            &dependency);

    CHECK_VK_RESULT_FATAL(
        g_device.createRenderPass(&renderPassInfo, g_allocator, &g_renderPass),
        "Failed to create render pass.");
  }

  void destroy_render_pass() {
    g_device.destroyRenderPass(g_renderPass, g_allocator);
  }

  void init_shaders() {
    g_baseShader.init("../resources/shaders/spv/shader.vert.spv",
                      "../resources/shaders/spv/shader.frag.spv");
  }

  void destroy_shaders() { g_baseShader.destroy(); }

  void init_framebuffers() {
    framebuffers.resize(imageViews.size());

    for (size_t i = 0; i < imageViews.size(); i++) {
      vk::ImageView attachments[] = {imageViews[i]};

      vk::FramebufferCreateInfo framebufferInfo(
          vk::FramebufferCreateFlags(), g_renderPass, 1, attachments,
          g_extent.width, g_extent.height, 1);

      CHECK_VK_RESULT_FATAL(g_device.createFramebuffer(&framebufferInfo,
                                                       g_allocator,
                                                       &framebuffers[i]),
                            "Failed to create framebuffer.");
    }
  }

  void destroy_framebuffers() {
    for (auto framebuffer : framebuffers)
      g_device.destroyFramebuffer(framebuffer, g_allocator);
  }

  void init_commandbuffers() {
    commandbuffers.resize(framebuffers.size());

    vk::CommandBufferAllocateInfo allocInfo(g_commandPool,
                                            vk::CommandBufferLevel::ePrimary,
                                            (uint32_t)commandbuffers.size());

    CHECK_VK_RESULT_FATAL(
        g_device.allocateCommandBuffers(&allocInfo, commandbuffers.data()),
        "Failed to allocate command buffers.");

    vk::CommandBufferBeginInfo beginInfo(
        vk::CommandBufferUsageFlagBits::eSimultaneousUse, nullptr);

    vk::Viewport viewport(0, 0, (float)g_extent.width, (float)g_extent.height,
                          0, 1);

    static constexpr const std::array<float, 4> clearColorArray = {
        {0.0f, 0.0f, 0.0f, 1.0f}};

    vk::ClearColorValue clearColorValue = clearColorArray;

    vk::ClearValue clearColor(clearColorValue);
    for (size_t i = 0; i < commandbuffers.size(); i++) {
      CHECK_VK_RESULT_FATAL(commandbuffers[i].begin(&beginInfo),
                            "Failed to begin recording command buffer.");

      vk::RenderPassBeginInfo renderPassInfo(
          g_renderPass, framebuffers[i], {{0, 0}, g_extent}, 1, &clearColor);

      commandbuffers[i].setViewport(
          0, 1,
          &viewport); // TODO buffers are recorded once so can't change viewport

      commandbuffers[i].beginRenderPass(&renderPassInfo,
                                        vk::SubpassContents::eInline);

	  //TODO mt record commands
      for (const auto &pair : objects) {
        commandbuffers[i].bindPipeline(
            vk::PipelineBindPoint::eGraphics, // TODO Compute shader support
            pair.first->pipeline);

        for (size_t j = 0; j < pair.second.size(); ++j) {
          pair.second[j].record(commandbuffers[i]);
        }
      }

      commandbuffers[i].endRenderPass();

      commandbuffers[i]
          .end(); // Strangely, it does returns void instead of vk::Result so no
                  // error checking is possible here
    }
  }

  void destroy_commandbuffers() {
    g_device.freeCommandBuffers(g_commandPool,
                                static_cast<uint32_t>(commandbuffers.size()),
                                commandbuffers.data());
  }

  void init_objects() {
    Object object;
    object.init(g_vertices, g_indices);

    objects[&g_baseShader].emplace_back(std::move(object));
  }

  void init(bool initMemory = true) {
    init_swapchain();

    init_render_pass();
    init_shaders();

    init_framebuffers();

    if (initMemory) {
      init_objects();
    }

    init_commandbuffers();
  }

  void destroy() {
    destroy_framebuffers();
    destroy_commandbuffers();

    destroy_render_pass();
    destroy_shaders();

    destroy_swapchain();
  }

  void refresh() {
    int width = 0, height = 0;
    while (width == 0 || height == 0) {
      WindowHandler::GetFramebufferSize(&width, &height);
      WindowHandler::Update();
    }

    g_device.waitIdle();

    destroy();
    init(false);
  }

} g_renderContext;

//-----------------------------------------------------------------------------
// DEBUG UTILS
//-----------------------------------------------------------------------------

#ifndef NDEBUG
static std::vector<vk::DebugUtilsMessengerEXT> g_debugMessengers;

static std::string
GetDebugMessagePrefix(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                      VkDebugUtilsMessageTypeFlagsEXT messageType) {
  std::string prefix = vk::to_string(
      static_cast<vk::DebugUtilsMessageSeverityFlagBitsEXT>(messageSeverity));

  prefix += " : ";
  prefix += vk::to_string(
      static_cast<vk::DebugUtilsMessageTypeFlagsEXT>(messageType));

  std::transform(prefix.begin(), prefix.end(), prefix.begin(), ::toupper);

  return prefix;
}

static std::string
FormatDebugMessage(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                   VkDebugUtilsMessageTypeFlagsEXT messageType,
                   const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  char *buf = (char *)malloc(strlen(callbackData->pMessage) + 500);

  sprintf(buf, "%s - Message ID Number %d, Message ID Name : %s\n\t%s",
          GetDebugMessagePrefix(messageSeverity, messageType).c_str(),
          callbackData->messageIdNumber, callbackData->pMessageIdName,
          callbackData->pMessage);

  std::string message = buf;
  free(buf);

  return message;
}

static std::string StringifyDebugMessageObjects(
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  if (callbackData->objectCount == 0)
    return std::string();

  char tmp[500];
  sprintf(tmp, "\n\n\t Objects - %d\n", callbackData->objectCount);

  std::string message(tmp);
  for (uint32_t object = 0; object < callbackData->objectCount; ++object) {
    char tmp_message[500];
    sprintf(
        tmp_message, "\t\t Object[%d] - Type %s, Value %p, Name \"%s\"\n",
        object,
        vk::to_string((vk::ObjectType)callbackData->pObjects[object].objectType)
            .c_str(),
        (void *)(callbackData->pObjects[object].objectHandle),
        callbackData->pObjects[object].pObjectName);

    message += tmp_message;
  }

  return message;
}

static std::string StringifyDebugMessageLabels(
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  if (callbackData->cmdBufLabelCount == 0)
    return std::string();

  char tmp[500];
  sprintf(tmp, "\n\n\t Command Buffer Labels - %d\n",
          callbackData->cmdBufLabelCount);

  std::string message(tmp);
  for (uint32_t label = 0; label < callbackData->cmdBufLabelCount; ++label) {
    char tmp_message[500];
    sprintf(tmp_message, "\t\t Label[%d] - %s { %f, %f, %f, %f}\n", label,
            callbackData->pCmdBufLabels[label].pLabelName,
            callbackData->pCmdBufLabels[label].color[0],
            callbackData->pCmdBufLabels[label].color[1],
            callbackData->pCmdBufLabels[label].color[2],
            callbackData->pCmdBufLabels[label].color[3]);

    message += tmp_message;
  }

  return message;
}

static std::string GetFullFormattedDebugMessage(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT *callbackData) {
  return FormatDebugMessage(messageSeverity, messageType, callbackData) +
         StringifyDebugMessageObjects(callbackData) +
         StringifyDebugMessageLabels(callbackData);
}

static void SetMainObjectsDebugNames() {
  g_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eInstance,
                                       (uint64_t)((VkInstance)g_instance),
                                       "Vulkan Instance"},
                                      g_dldy);

  g_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eSurfaceKHR,
                                       (uint64_t)((VkSurfaceKHR)g_surface),
                                       "Window Surface"},
                                      g_dldy);

  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::ePhysicalDevice,
       (uint64_t)((VkPhysicalDevice)g_physicalDevice),
       (std::string("Physical Device - ") +
        g_physicalDevice.getProperties().deviceName)
           .c_str()},
      g_dldy);
  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eDevice, (uint64_t)((VkDevice)g_device),
       (std::string("Logical Device - ") +
        g_physicalDevice.getProperties().deviceName)
           .c_str()},
      g_dldy);

  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eQueue, (uint64_t)((VkQueue)g_graphicsQueue.handle),
       "Graphics Queue"},
      g_dldy);
  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eQueue, (uint64_t)((VkQueue)g_presentQueue.handle),
       "Present Queue"},
      g_dldy);

  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eSwapchainKHR,
       (uint64_t)((VkSwapchainKHR)g_renderContext.swapchain), "Swapchain"},
      g_dldy);

  for (size_t i = 0; i < g_debugMessengers.size(); ++i)
    g_device.setDebugUtilsObjectNameEXT(
        {vk::ObjectType::eDebugUtilsMessengerEXT,
         (uint64_t)((VkDebugUtilsMessengerEXT)g_debugMessengers[i]),
         g_debugMessengersInfos[i].name},
        g_dldy);
}
#endif

//-----------------------------------------------------------------------------
// DRAW
//-----------------------------------------------------------------------------

static std::array<vk::Semaphore, MAX_IN_FLIGHT_FRAMES>
    g_imageAvailableSemaphores;
static std::array<vk::Semaphore, MAX_IN_FLIGHT_FRAMES>
    g_renderFinishedSemaphores;

static std::array<vk::Fence, MAX_IN_FLIGHT_FRAMES> g_inFlightFences;

static size_t g_currentFrame = 0;

static void Draw() {
  g_device.waitForFences(1, &g_inFlightFences[g_currentFrame], VK_TRUE,
                         std::numeric_limits<uint64_t>::max());

  uint32_t imageIndex;
  vk::Result result = g_device.acquireNextImageKHR(
      g_renderContext.swapchain, std::numeric_limits<uint64_t>::max(),
      g_imageAvailableSemaphores[g_currentFrame], nullptr, &imageIndex);

  if (result == vk::Result::eErrorOutOfDateKHR) {
    g_renderContext.refresh();
    return;
  } else if (result != vk::Result::eSuccess &&
             result != vk::Result::eSuboptimalKHR) {
    std::string err = "[FATAL]";
    err += "Failed to acquire Swapchain image.";
    err += " : ";
    err += vk::to_string(result);

    throw std::runtime_error(err.c_str());
  }

  vk::PipelineStageFlags waitStage =
      vk::PipelineStageFlagBits::eColorAttachmentOutput;

  vk::SubmitInfo submitInfo(1, &g_imageAvailableSemaphores[g_currentFrame],
                            &waitStage, 1,
                            &g_renderContext.commandbuffers[imageIndex], 1,
                            &g_renderFinishedSemaphores[g_currentFrame]);

  g_device.resetFences(1, &g_inFlightFences[g_currentFrame]);

  CHECK_VK_RESULT_FATAL(g_graphicsQueue.handle.submit(
                            1, &submitInfo, g_inFlightFences[g_currentFrame]),
                        "Failed to submit draw command buffer.");

  vk::PresentInfoKHR presentInfo(1, &g_renderFinishedSemaphores[g_currentFrame],
                                 1, &g_renderContext.swapchain, &imageIndex,
                                 nullptr);

  result = g_presentQueue.handle.presentKHR(&presentInfo);
  if (result == vk::Result::eErrorOutOfDateKHR ||
      result == vk::Result::eSuboptimalKHR || g_framebufferResized) {
    g_framebufferResized = false;
    g_renderContext.refresh();
  } else
    CHECK_VK_RESULT_FATAL(result, "Failed to present Swapchain image.");

  g_currentFrame = (g_currentFrame + 1) % MAX_IN_FLIGHT_FRAMES;
}

//-----------------------------------------------------------------------------
// INITIALIZATION
//-----------------------------------------------------------------------------

static void CreateInstance() {
  vk::ApplicationInfo appInfo("Renderer", VK_MAKE_VERSION(1, 0, 0), "No Engine",
                              VK_MAKE_VERSION(1, 0, 0), VK_API_VERSION_1_1);

  vk::InstanceCreateInfo createInfo;
  createInfo.pApplicationInfo = &appInfo;

  const char **pExtensions = nullptr;

  uint32_t requiredExtensionsCount =
      WindowHandler::GetRequiredInstanceExtensions(pExtensions);
  std::vector<const char *> extensions(pExtensions,
                                       pExtensions + requiredExtensionsCount);

  extensions.insert(extensions.end(), g_instanceExtensions.begin(),
                    g_instanceExtensions.end());

  createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
  createInfo.ppEnabledExtensionNames = extensions.data();

#ifndef NDEBUG
  createInfo.enabledLayerCount =
      static_cast<uint32_t>(g_validationLayers.size());
  createInfo.ppEnabledLayerNames = g_validationLayers.data();
#else
  createInfo.enabledLayerCount = 0;
#endif

  CHECK_VK_RESULT_FATAL(
      vk::createInstance(&createInfo, g_allocator, &g_instance),
      "Failed to init vulkan instance");

  g_dldy.init(g_instance);

#ifndef NDEBUG
  for (size_t i = 0; i < g_debugMessengersInfos.size(); ++i)
    g_debugMessengers.push_back(g_instance.createDebugUtilsMessengerEXT(
        g_debugMessengersInfos[i].MakeCreateInfo(), g_allocator, g_dldy));
#endif
}

static void InitDevice() {
  const std::vector<vk::PhysicalDevice> devices =
      g_instance.enumeratePhysicalDevices();

  if (devices.empty())
    throw std::runtime_error("failed to find GPUs with Vulkan support!");

  {
    // Use an ordered map to automatically sort candidates by increasing score
    std::multimap<int, vk::PhysicalDevice> candidates;

    for (const auto &device : devices) {
      int score = RateDeviceSuitability(device);
      candidates.insert(std::make_pair(score, device));
    }

    // Check if the best candidate is suitable at all
    if (candidates.rbegin()->first > 0)
      g_physicalDevice = candidates.rbegin()->second;
    else
      throw std::runtime_error("Failed to find a suitable GPU");
  }

  std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
  const QueueFamilyIndices indices = GetQueueFamilies(g_physicalDevice);
  const std::set<int> uniqueQueueFamilies = {indices.graphicsFamily,
                                             indices.presentFamily};

  float queuePriority = 1.0f;
  for (int queueFamily : uniqueQueueFamilies) {
    vk::DeviceQueueCreateInfo queueCreateInfo(vk::DeviceQueueCreateFlags(),
                                              queueFamily, 1, &queuePriority);

    queueCreateInfos.push_back(queueCreateInfo);
  }

  vk::PhysicalDeviceFeatures deviceFeatures = {};

  vk::DeviceCreateInfo createInfo(
      vk::DeviceCreateFlags(), static_cast<uint32_t>(queueCreateInfos.size()),
      queueCreateInfos.data(),
#ifndef NDEBUG
      static_cast<uint32_t>(g_validationLayers.size()),
      g_validationLayers.data(),
#else
      0, nullptr,
#endif
      static_cast<uint32_t>(g_deviceExtensions.size()),
      g_deviceExtensions.data(), &deviceFeatures);

  CHECK_VK_RESULT_FATAL(
      g_physicalDevice.createDevice(&createInfo, g_allocator, &g_device),
      "Failed to create logical device");

  g_dldy.init(g_instance, g_device);

  g_graphicsQueue.index = 0;
  g_graphicsQueue.handle =
      g_device.getQueue(indices.graphicsFamily, g_graphicsQueue.index);

  g_presentQueue.index = 0;
  g_presentQueue.handle =
      g_device.getQueue(indices.presentFamily, g_presentQueue.index);
}

static void InitCommandPools() {
  QueueFamilyIndices queueFamilyIndices = GetQueueFamilies(g_physicalDevice);

  vk::CommandPoolCreateInfo poolInfo(vk::CommandPoolCreateFlags(),
                                     queueFamilyIndices.graphicsFamily);

  vk::CommandPoolCreateInfo stagingPoolInfo(
      vk::CommandPoolCreateFlagBits::eTransient,
      queueFamilyIndices.graphicsFamily);

  CHECK_VK_RESULT_FATAL(
      g_device.createCommandPool(&poolInfo, g_allocator, &g_commandPool),
      "Failed to create command pool.");

  CHECK_VK_RESULT_FATAL(g_device.createCommandPool(&stagingPoolInfo,
                                                   g_allocator,
                                                   &g_stagingCommandPool),
                        "Failed to create staging command pool.");
}

static void InitVMA() {
  VmaAllocatorCreateInfo allocatorInfo = {};
  allocatorInfo.physicalDevice = g_physicalDevice;
  allocatorInfo.device = g_device;

  vmaCreateAllocator(&allocatorInfo, &g_vmaAllocator);
}

static void InitSyncBarriers() {
  vk::SemaphoreCreateInfo semaphoreInfo = {};

  vk::FenceCreateInfo fenceInfo = {vk::FenceCreateFlagBits::eSignaled};

  for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++) {
    CHECK_VK_RESULT_FATAL(
        g_device.createSemaphore(&semaphoreInfo, g_allocator,
                                 &g_imageAvailableSemaphores[i]),
        "Failed to create semaphores.");
    CHECK_VK_RESULT_FATAL(
        g_device.createSemaphore(&semaphoreInfo, g_allocator,
                                 &g_renderFinishedSemaphores[i]),
        "Failed to create semaphores.");

    CHECK_VK_RESULT_FATAL(
        g_device.createFence(&fenceInfo, g_allocator, &g_inFlightFences[i]),
        "Failed to create semaphores.");
  }
}

static void InitVulkan() {
  CreateInstance();
  CHECK_VK_RESULT_FATAL((vk::Result)WindowHandler::CreateSurface(
                            (VkInstance)g_instance,
                            (VkAllocationCallbacks *)g_allocator,
                            (VkSurfaceKHR *)&g_surface),
                        "Failed to create window surface");
  InitDevice();

  InitCommandPools();
  InitVMA();

  g_renderContext.init();

  InitSyncBarriers();

#ifndef NDEBUG
  SetMainObjectsDebugNames();
#endif
}

//-----------------------------------------------------------------------------
// USER FUNCTIONS
//-----------------------------------------------------------------------------

void Init(unsigned int width, unsigned int height) {
  assert(width != 0 && height != 0);

  TRY_CATCH_BLOCK("Failed to init renderer",

                  WindowHandler::Init(width, height);
                  InitVulkan(););
}

bool Update() {
  TRY_CATCH_BLOCK("Failed to update renderer",

                  bool signal = WindowHandler::Update();
                  Draw();

                  return signal;);
}

void Shutdown() {
  TRY_CATCH_BLOCK(
      "Failed to shutdown renderer",
      for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++) {
        g_device.destroySemaphore(g_imageAvailableSemaphores[i], g_allocator);
        g_device.destroySemaphore(g_renderFinishedSemaphores[i], g_allocator);

        g_device.destroyFence(g_inFlightFences[i], g_allocator);
      }

      g_renderContext.destroy();
      g_renderContext.objects.clear();

      vmaDestroyAllocator(g_vmaAllocator);

      g_device.destroyCommandPool(g_commandPool, g_allocator);
      g_device.destroyCommandPool(g_stagingCommandPool, g_allocator);

      g_device.destroy(g_allocator);

#ifndef NDEBUG
      for (size_t i = 0; i < g_debugMessengers.size(); ++i)
          g_instance.destroyDebugUtilsMessengerEXT(g_debugMessengers[i],
                                                   g_allocator, g_dldy);
#endif

      g_instance.destroySurfaceKHR(g_surface, g_allocator);
      g_instance.destroy(g_allocator);

      WindowHandler::Shutdown(););
}

void Run(unsigned int width, unsigned int height) {
  Init(width, height);

  while (Update())
    ;
  g_device.waitIdle();

  Shutdown();
}

void SetPresentMode(PresentMode presentMode) {
  g_renderContext.requiredPresentMode =
      static_cast<vk::PresentModeKHR>(presentMode);
  g_renderContext.refresh();
}
} // namespace Renderer
