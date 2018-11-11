#define VMA_IMPLEMENTATION
#include "memory.hpp"

#include "debug_tools.hpp"

//-----------------------------------------------------------------------------
// ALLOCATOR
//-----------------------------------------------------------------------------

VmaAllocator g_handle = nullptr;
vk::CommandPool g_stagingCommandPool = nullptr;

vk::Device g_device = nullptr;
vk::AllocationCallbacks *g_allocationCallbacks = nullptr;

namespace Allocator {
void Init(vk::PhysicalDevice physicalDevice, vk::Device deviceHandle,
          int stagingQueueIndex,
          vk::AllocationCallbacks *pAllocationCallbacks) {
  g_device = deviceHandle;
  g_allocationCallbacks = pAllocationCallbacks;

  VmaAllocatorCreateInfo allocatorInfo = {};
  allocatorInfo.physicalDevice = physicalDevice;
  allocatorInfo.device = g_device;

  vmaCreateAllocator(&allocatorInfo, &g_handle);

  vk::CommandPoolCreateInfo stagingPoolInfo(
      vk::CommandPoolCreateFlagBits::eTransient, stagingQueueIndex);

  CHECK_VK_RESULT_FATAL(g_device.createCommandPool(&stagingPoolInfo,
                                                   g_allocationCallbacks,
                                                   &g_stagingCommandPool),
                        "Failed to create staging command pool.");
};

void Destroy() {
  vmaDestroyAllocator(g_handle);
  g_device.destroyCommandPool(g_stagingCommandPool, g_allocationCallbacks);
}
} // namespace Allocator

//-----------------------------------------------------------------------------
// BUFFER
//-----------------------------------------------------------------------------

Buffer::Buffer(Buffer &&other) {
  handle = other.handle;
  other.handle = nullptr;

  allocation = std::move(other.allocation);
}

Buffer::~Buffer() {
  if (handle)
    vmaDestroyBuffer(g_handle, handle, allocation);
}

void Buffer::allocate(VkDeviceSize allocationSize, vk::BufferUsageFlags usage,
                      vk::MemoryPropertyFlags properties) {
#ifndef NDEBUG
  if (handle)
    printf("[WARNING] Allocating already allocated buffer. This probably "
           "means memory leaks. (Address = %p)",
           (VkBuffer)handle);
#endif

  size = allocationSize;

  VkBufferCreateInfo bufferInfo = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  bufferInfo.size = size;
  bufferInfo.usage = (VkBufferUsageFlags)usage;
  bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocInfo = {};
  allocInfo.usage = VMA_MEMORY_USAGE_UNKNOWN;
  allocInfo.requiredFlags = (VkMemoryPropertyFlags)properties;

  CHECK_VK_RESULT_FATAL(vmaCreateBuffer(g_handle, &bufferInfo, &allocInfo,
                                        (VkBuffer *)&handle, &allocation,
                                        nullptr),
                        "Failed to create vertex buffer.");
}

// TODO Staging queue (TRANSFER_BIT)
void Buffer::copy_to(Buffer &dstBuffer, VkDeviceSize copySize,
                     vk::Device device, vk::Queue queue) const {
  vk::CommandBufferAllocateInfo allocInfo(g_stagingCommandPool,
                                          vk::CommandBufferLevel::ePrimary, 1);

  vk::CommandBuffer commandBuffer;
  device.allocateCommandBuffers(&allocInfo, &commandBuffer);

  vk::CommandBufferBeginInfo beginInfo(
      vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  commandBuffer.begin(&beginInfo);

  vk::BufferCopy copyRegion(0, // TODO src offset
                            0, // TODO dst offset
                            copySize);

  commandBuffer.copyBuffer(handle, dstBuffer.handle, 1, &copyRegion);

  commandBuffer.end();

  vk::SubmitInfo submitInfo = {};
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandBuffer;

  queue.submit(1, &submitInfo, nullptr);
  queue.waitIdle();

  device.freeCommandBuffers(g_stagingCommandPool, 1, &commandBuffer);
}

void Buffer::write(const void *data, VkDeviceSize writeSize,
                   VkDeviceSize offset) const {

  if (writeSize + offset > size) {
#ifndef NDEBUG
    printf("[WARNING] Specified write to buffer out of allocation bounds. "
           "Data might be corrupted. (Address = %p)",
           (VkBuffer)handle);
#endif
    if (offset >= size)
      return;

    writeSize = size - offset;
  }

  void *mappedMemory;
  vmaMapMemory(g_handle, allocation, &mappedMemory);
  memcpy((char *)mappedMemory + offset, data, (size_t)writeSize);
  vmaUnmapMemory(g_handle, allocation);
}

Buffer &Buffer::operator=(Buffer &&other) {
  handle = other.handle;
  other.handle = nullptr;

  allocation = std::move(other.allocation);

  return *this;
}

//-----------------------------------------------------------------------------
// IMAGE
//-----------------------------------------------------------------------------

static uint8_t GetPixelSizeFromFormat(vk::Format format) {
  switch (format) {
  case vk::Format::eR8G8B8A8Unorm:
    return 4;
  default:
    printf("[WARNING] Unsupported image format specified (%s).",
           vk::to_string(format).c_str());
    return 0;
  }
}

Image::Image(Image &&other) {
  handle = other.handle;
  other.handle = nullptr;

  allocation = std::move(other.allocation);
}

Image::~Image() {
  if (handle)
    vmaDestroyImage(g_handle, handle, allocation);
}

void Image::allocate(uint32_t texWidth, uint32_t texHeight, vk::Format format,
                     vk::ImageTiling tiling, vk::ImageUsageFlags usage,
                     vk::MemoryPropertyFlags properties) {
#ifndef NDEBUG
  if (handle)
    printf("[WARNING] Allocating already allocated image. This probably "
           "means memory leaks. (Address = %p)",
           (VkImage)handle);
#endif

  size = texWidth * texHeight * GetPixelSizeFromFormat(format);

  vk::ImageCreateInfo imageInfo{
      vk::ImageCreateFlags(),
      vk::ImageType::e2D,
      format,
      {texWidth, texHeight, 1},
      1, // TODO Support mipmapping
      1,
      vk::SampleCountFlagBits::e1, // TODO Support multisampling image
                                   // attachments
      tiling,
      usage,
      vk::SharingMode::eExclusive, // TODO Support Concurrent access
      0,
      nullptr,
      vk::ImageLayout::eUndefined};

  VmaAllocationCreateInfo allocInfo = {};
  allocInfo.usage = VMA_MEMORY_USAGE_UNKNOWN;
  allocInfo.requiredFlags = (VkMemoryPropertyFlags)properties;

  CHECK_VK_RESULT_FATAL(
      vmaCreateImage(g_handle, (VkImageCreateInfo *)&imageInfo, &allocInfo,
                     (VkImage *)&handle, &allocation, nullptr),
      "Failed to create image.");
}

Image &Image::operator=(Image &&other) {
  handle = other.handle;
  other.handle = nullptr;

  allocation = std::move(other.allocation);

  return *this;
}
