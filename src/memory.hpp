#pragma once

#include <vulkan/vulkan.hpp>

#include "vk_mem_alloc.h"

//-----------------------------------------------------------------------------
// MEMORY
//-----------------------------------------------------------------------------

struct Allocator {
private:
  static VmaAllocator handle;
  static vk::CommandPool stagingCommandPool;

  static vk::Device device;
  static vk::AllocationCallbacks *allocationCallbacks;

public:
  static VmaAllocator GetHandle() { return handle; }
  static vk::CommandPool GetStagingCommandPool();
  static void Init(vk::PhysicalDevice physicalDevice, vk::Device deviceHandle,
	  int stagingQueueIndex,
	  vk::AllocationCallbacks *pAllocationCallbacks);

  static void Destroy();
};

//-----------------------------------------------------------------------------
// BUFFER
//-----------------------------------------------------------------------------

struct Buffer {
private:
  vk::Buffer handle;
  VmaAllocation allocation;
  size_t size = 0;

public:
  Buffer() = default;

  Buffer(const Buffer &other) = delete;
  Buffer(Buffer &&other);

  ~Buffer();

  const vk::Buffer &get_handle() const { return handle; }

  void allocate(VkDeviceSize allocationSize, vk::BufferUsageFlags usage,
	  vk::MemoryPropertyFlags properties);

  // TODO Staging queue (TRANSFER_BIT)
  void copy_to(Buffer &dstBuffer, VkDeviceSize copySize, vk::Device device,
	  vk::Queue queue) const;

  void write(const void *data, VkDeviceSize writeSize,
	  VkDeviceSize offset = 0) const;

  Buffer &operator=(const Buffer &o) = delete;
  Buffer &operator=(Buffer &&other);
};

//-----------------------------------------------------------------------------
// IMAGE
//-----------------------------------------------------------------------------

struct Image {
private:
  vk::Image handle;
  VmaAllocation allocation;
  size_t size = 0;

public:
  Image() = default;

  Image(const Image &other) = delete;
  Image(Image &&other);

  ~Image();

  const vk::Image &get_handle() const { return handle; }

  void allocate(uint32_t texWidth, uint32_t texHeight, vk::Format format,
	  vk::ImageTiling tiling, vk::ImageUsageFlags usage,
	  vk::MemoryPropertyFlags properties);

  Image &operator=(const Image &o) = delete;
  Image &operator=(Image &&other);
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
