#define VMA_IMPLEMENTATION
#include "memory.hpp"

#include "configuration_helper.hpp"
#include "global_context.hpp"

//-----------------------------------------------------------------------------
// ALLOCATOR
//-----------------------------------------------------------------------------

static VmaAllocator g_handle = nullptr;

static vk::CommandPool g_stagingCommandPool = nullptr;
static Queue g_stagingQueue;

static vk::CommandBuffer BeginSingleTimeCommand() {
  vk::CommandBufferAllocateInfo allocInfo(g_stagingCommandPool,
                                          vk::CommandBufferLevel::ePrimary, 1);

  vk::CommandBuffer commandbuffer;
  g_device.allocateCommandBuffers(&allocInfo, &commandbuffer);

  vk::CommandBufferBeginInfo beginInfo(
      vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  commandbuffer.begin(&beginInfo);

  return commandbuffer;
}

static void EndSingleTimeCommand(vk::CommandBuffer commandbuffer) {
  commandbuffer.end();

  vk::SubmitInfo submitInfo = {};
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandbuffer;

  g_stagingQueue.handle.submit(1, &submitInfo, nullptr);
  g_stagingQueue.handle.waitIdle();

  g_device.freeCommandBuffers(g_stagingCommandPool, 1, &commandbuffer);
}

namespace Allocator {
void Init(::Queue stagingQueue) {
  g_stagingQueue = stagingQueue;

  VmaAllocatorCreateInfo allocatorInfo = {};
  allocatorInfo.physicalDevice = g_physicalDevice;
  allocatorInfo.device = g_device;

  vmaCreateAllocator(&allocatorInfo, &g_handle);

  vk::CommandPoolCreateInfo stagingPoolInfo(
      vk::CommandPoolCreateFlagBits::eTransient, stagingQueue.index);

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
           "means memory leaks. (Address = %p)\n",
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
void Buffer::copy_to(Buffer &dstBuffer, VkDeviceSize copySize) const {
  vk::BufferCopy copyRegion(0, // TODO src offset
                            0, // TODO dst offset
                            copySize);

  vk::CommandBuffer commandbuffer = BeginSingleTimeCommand();
  commandbuffer.copyBuffer(handle, dstBuffer.handle, 1, &copyRegion);

  EndSingleTimeCommand(commandbuffer);
}

void Buffer::write(const void *data, VkDeviceSize writeSize,
                   VkDeviceSize offset) const {

  if (writeSize + offset > size) {
#ifndef NDEBUG
    printf("[WARNING] Specified write to buffer out of allocation bounds. "
           "Data might be corrupted. (Address = %p)\n",
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
    printf("[WARNING] Unsupported image format specified (%s).\n",
           vk::to_string(format).c_str());
    return 0;
  }
}

Image::Image(Image &&other) {
  handle = other.handle;
  other.handle = nullptr;

  allocation = std::move(other.allocation);
  size = other.size;

  layout = other.layout;
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
           "means memory leaks. (Address = %p)\n",
           (VkImage)handle);
#endif

  size = texWidth * texHeight * GetPixelSizeFromFormat(format);
  layout = vk::ImageLayout::eUndefined;

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
      layout};

  VmaAllocationCreateInfo allocInfo = {};
  allocInfo.usage = VMA_MEMORY_USAGE_UNKNOWN;
  allocInfo.requiredFlags = (VkMemoryPropertyFlags)properties;

  CHECK_VK_RESULT_FATAL(
      vmaCreateImage(g_handle, (VkImageCreateInfo *)&imageInfo, &allocInfo,
                     (VkImage *)&handle, &allocation, nullptr),
      "Failed to create image.");
}

void Image::transition_layout(vk::Format format, vk::ImageLayout newLayout) {
  vk::CommandBuffer commandbuffer = BeginSingleTimeCommand();

  vk::ImageSubresourceRange subresourceRange;
  subresourceRange.aspectMask =
      vk::ImageAspectFlagBits::eColor; // TODO Support other aspect masks
  subresourceRange.baseMipLevel = 0;
  subresourceRange.levelCount = 1;
  subresourceRange.baseArrayLayer = 0;
  subresourceRange.layerCount = 1;

  vk::ImageMemoryBarrier barrier;
  barrier.oldLayout = layout;
  barrier.newLayout = newLayout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = handle;
  barrier.subresourceRange = subresourceRange;

  vk::PipelineStageFlags srcStage;
  vk::PipelineStageFlags dstStage;

  if (layout == vk::ImageLayout::eUndefined &&
      newLayout == vk::ImageLayout::eTransferDstOptimal) {
    barrier.srcAccessMask = vk::AccessFlagBits(0);
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;

    srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
    dstStage = vk::PipelineStageFlagBits::eTransfer;
  } else if (layout == vk::ImageLayout::eTransferDstOptimal &&
             newLayout == vk::ImageLayout::eShaderReadOnlyOptimal) {
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;

    srcStage = vk::PipelineStageFlagBits::eTransfer;
    dstStage = vk::PipelineStageFlagBits::eFragmentShader;
  } else {
    std::string msg = "Unsupported layout transition."
                      " (" +
                      vk::to_string(layout) + " -> " +
                      vk::to_string(newLayout) + ")";

    throw std::invalid_argument(msg.c_str());
  }

  commandbuffer.pipelineBarrier(srcStage, dstStage,
                                vk::DependencyFlagBits::eByRegion, 0, nullptr,
                                0, nullptr, 1, &barrier);

  EndSingleTimeCommand(commandbuffer);

  layout = newLayout;
}

// TODO Make it more flexible
void Image::write_from_buffer(vk::Buffer buffer, uint32_t width,
                              uint32_t height) {
  vk::CommandBuffer commandbuffer = BeginSingleTimeCommand();

  vk::BufferImageCopy region;
  region.bufferOffset = 0;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;

  vk::ImageSubresourceLayers imageSubresource;
  imageSubresource.aspectMask =
      vk::ImageAspectFlagBits::eColor; // TODO Support other aspect masks
  imageSubresource.mipLevel = 0;
  imageSubresource.baseArrayLayer = 0;
  imageSubresource.layerCount = 1;

  region.imageSubresource = imageSubresource;

  region.imageOffset = vk::Offset3D(0, 0, 0);
  region.imageExtent = vk::Extent3D(width, height, 1);

  commandbuffer.copyBufferToImage(
      buffer, handle, vk::ImageLayout::eTransferDstOptimal, 1, &region);

  EndSingleTimeCommand(commandbuffer);
}

Image &Image::operator=(Image &&other) {
  handle = other.handle;
  other.handle = nullptr;

  allocation = std::move(other.allocation);
  size = other.size;

  layout = other.layout;

  return *this;
}
