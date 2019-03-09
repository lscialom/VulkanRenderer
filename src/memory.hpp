#pragma once

#include <vulkan/vulkan.hpp>

#include "vk_mem_alloc.h"

namespace Renderer {

//-----------------------------------------------------------------------------
// ALLOCATOR
//-----------------------------------------------------------------------------

namespace Allocator {
void Init();

void SetTransferQueue(struct Queue queue);
void WaitForTransferQueue();

void Destroy();
}; // namespace Allocator

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
  void copy_to(Buffer &dstBuffer, VkDeviceSize copySize) const;

  void write(const void *data, VkDeviceSize writeSize,
             VkDeviceSize offset = 0) const;

  size_t get_size() const { return size; }

  Buffer &operator=(const Buffer &o) = delete;
  Buffer &operator=(Buffer &&other);
};

//-----------------------------------------------------------------------------
// IMAGE
//-----------------------------------------------------------------------------

struct Image {
private:
  vk::Image handle;
  vk::ImageView view;
  VmaAllocation allocation;
  size_t size = 0;

  vk::ImageAspectFlags aspect;

public:
  Image() = default;

  Image(const Image &other) = delete;
  Image(Image &&other);

  ~Image();

  const vk::Image &get_handle() const { return handle; }
  const vk::ImageView &get_view() const { return view; }

  void
  allocate(uint32_t texWidth, uint32_t texHeight, vk::Format format,
           vk::ImageTiling tiling, vk::ImageUsageFlags usage,
           vk::MemoryPropertyFlags properties,
           vk::ImageAspectFlags aspectFlags = vk::ImageAspectFlagBits::eColor);
  void free();

  void transition_layout(vk::ImageLayout oldLayout, vk::ImageLayout newLayout);

  void write_from_buffer(vk::Buffer buffer, uint32_t width, uint32_t height);

  Image &operator=(const Image &o) = delete;
  Image &operator=(Image &&other);
};

//-----------------------------------------------------------------------------
// ATTACHMENT
//-----------------------------------------------------------------------------

struct Attachment {
private:
  Image image;

public:
  vk::Format requiredFormat;
  uint32_t arrayIndex = 0; // For user

  void
  init(uint32_t texWidth, uint32_t texHeight,
       vk::ImageTiling tiling = vk::ImageTiling::eOptimal,
       vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eColorAttachment |
                                   vk::ImageUsageFlagBits::eSampled,
       vk::MemoryPropertyFlags properties =
           vk::MemoryPropertyFlagBits::eDeviceLocal,
       vk::ImageAspectFlags aspectFlags = vk::ImageAspectFlagBits::eColor,
       vk::ImageLayout dstLayout = vk::ImageLayout::eShaderReadOnlyOptimal) {

    image.allocate(texWidth, texHeight, requiredFormat, tiling, usage,
                   properties, aspectFlags);
    image.transition_layout(vk::ImageLayout::eUndefined, dstLayout);
  }

  void
  init(uint32_t texWidth, uint32_t texHeight, vk::Format format,
       vk::ImageTiling tiling = vk::ImageTiling::eOptimal,
       vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eColorAttachment |
                                   vk::ImageUsageFlagBits::eSampled,
       vk::MemoryPropertyFlags properties =
           vk::MemoryPropertyFlagBits::eDeviceLocal,
       vk::ImageAspectFlags aspectFlags = vk::ImageAspectFlagBits::eColor,
       vk::ImageLayout dstLayout = vk::ImageLayout::eShaderReadOnlyOptimal) {

    requiredFormat = format;
    init(texWidth, texHeight, tiling, usage, properties, aspectFlags,
         dstLayout);
  }

  void destroy() { image.free(); }

  const vk::ImageView &get_image_view() const { return image.get_view(); }
  vk::AttachmentDescription
  make_attachment_description(vk::ImageLayout initialLayout,
                              vk::ImageLayout finalLayout) {

    vk::AttachmentDescription attachmentDescription = {};

    attachmentDescription.initialLayout = initialLayout;
    attachmentDescription.finalLayout = finalLayout;
    attachmentDescription.format = requiredFormat;
    attachmentDescription.loadOp = vk::AttachmentLoadOp::eClear;
    attachmentDescription.storeOp = vk::AttachmentStoreOp::eStore;
    attachmentDescription.stencilLoadOp = vk::AttachmentLoadOp::eDontCare;
    attachmentDescription.stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
    attachmentDescription.samples = vk::SampleCountFlagBits::e1;

    return attachmentDescription;
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

} // namespace Renderer
