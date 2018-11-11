#include <vulkan/vulkan.hpp>

#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"

//-----------------------------------------------------------------------------
// MEMORY
//-----------------------------------------------------------------------------

static VmaAllocator g_vmaAllocator;
static vk::CommandPool g_stagingCommandPool;

static void InitAllocator(vk::PhysicalDevice physicalDevice,
                          vk::Device device) {
  VmaAllocatorCreateInfo allocatorInfo = {};
  allocatorInfo.physicalDevice = physicalDevice;
  allocatorInfo.device = device;

  vmaCreateAllocator(&allocatorInfo, &g_vmaAllocator);
}

static void DestroyAllocator() { vmaDestroyAllocator(g_vmaAllocator); }

struct Buffer {
private:
  vk::Buffer handle;
  VmaAllocation allocation;
  size_t size = 0;

public:
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

  const vk::Buffer &get_handle() const { return handle; }

  void allocate(VkDeviceSize allocationSize, vk::BufferUsageFlags usage,
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

    CHECK_VK_RESULT_FATAL(vmaCreateBuffer(g_vmaAllocator, &bufferInfo,
                                          &allocInfo, (VkBuffer *)&handle,
                                          &allocation, nullptr),
                          "Failed to create vertex buffer.");
  }
  
  //TODO Staging queue (TRANSFER_BIT)
  void copy_to(Buffer &dstBuffer, VkDeviceSize copySize, vk::Device device,
               vk::Queue queue) const {
    vk::CommandBufferAllocateInfo allocInfo(
        g_stagingCommandPool, vk::CommandBufferLevel::ePrimary, 1);

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

  void write(const void *data, VkDeviceSize writeSize,
             VkDeviceSize offset = 0) const {

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
    vmaMapMemory(g_vmaAllocator, allocation, &mappedMemory);
    memcpy((char *)mappedMemory + offset, data, (size_t)writeSize);
    vmaUnmapMemory(g_vmaAllocator, allocation);
  }

  Buffer &operator=(const Buffer &o) = delete;
  Buffer &operator=(Buffer &&other) {
    handle = other.handle;
    other.handle = nullptr;

    allocation = std::move(other.allocation);

    return *this;
  }
};

struct Image {
private:
  vk::Image handle;
  VmaAllocation allocation;
  size_t size = 0;

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

public:
  Image() = default;

  Image(const Image &other) = delete;
  Image(Image &&other) {
    handle = other.handle;
    other.handle = nullptr;

    allocation = std::move(other.allocation);
  }

  ~Image() {
    if (handle)
      vmaDestroyImage(g_vmaAllocator, handle, allocation);
  }

  const vk::Image &get_handle() const { return handle; }

  void allocate(uint32_t texWidth, uint32_t texHeight, vk::Format format,
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
        vmaCreateImage(g_vmaAllocator, (VkImageCreateInfo *)&imageInfo,
                       &allocInfo, (VkImage *)&handle, &allocation, nullptr),
        "Failed to create image.");
  }

  Image &operator=(const Image &o) = delete;
  Image &operator=(Image &&other) {
    handle = other.handle;
    other.handle = nullptr;

    allocation = std::move(other.allocation);

    return *this;
  }
};

// static void CreateVertexBuffer(const std::vector<Vertex> &vertexBuffer,
//                               Buffer &buffer) {
//  VkDeviceSize bufferSize = sizeof(Vertex) * vertexBuffer.size();
//
//  Buffer stagingBuffer;
//  stagingBuffer.allocate(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
//                         vk::MemoryPropertyFlagBits::eHostVisible |
//                             vk::MemoryPropertyFlagBits::eHostCoherent);
//
//  stagingBuffer.write(vertexBuffer.data(), bufferSize);
//
//  buffer.allocate(bufferSize,
//                  vk::BufferUsageFlagBits::eTransferDst |
//                      vk::BufferUsageFlagBits::eVertexBuffer,
//                  vk::MemoryPropertyFlagBits::eDeviceLocal);
//
//  stagingBuffer.copy_to(buffer, bufferSize);
//}
//
// static void
// CreateIndexBuffer(const std::vector<VERTEX_INDICES_TYPE> &indexBuffer,
//                  Buffer &buffer) {
//  VkDeviceSize bufferSize = sizeof(VERTEX_INDICES_TYPE) * indexBuffer.size();
//
//  Buffer stagingBuffer;
//  stagingBuffer.allocate(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
//                         vk::MemoryPropertyFlagBits::eHostVisible |
//                             vk::MemoryPropertyFlagBits::eHostCoherent);
//
//  stagingBuffer.write(indexBuffer.data(), bufferSize);
//
//  buffer.allocate(bufferSize,
//                  vk::BufferUsageFlagBits::eTransferDst |
//                      vk::BufferUsageFlagBits::eIndexBuffer,
//                  vk::MemoryPropertyFlagBits::eDeviceLocal);
//
//  stagingBuffer.copy_to(buffer, bufferSize);
//}
