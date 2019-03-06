#define VMA_IMPLEMENTATION
#include "memory.hpp"

#include "configuration_helper.hpp"
#include "global_context.hpp"

namespace Renderer {

//-----------------------------------------------------------------------------
// ALLOCATOR
//-----------------------------------------------------------------------------

static VmaAllocator vmaAllocator = nullptr;

static vk::CommandPool stagingCommandPool = nullptr;
static Queue transferQueue;

static std::array<vk::CommandBuffer, 3> commandbuffersAsync;
static std::array<vk::Fence, 3> cbAsyncFences;

static uint8_t lastCommandbufferUsedID = commandbuffersAsync.size() - 1;

static vk::CommandBuffer BeginSingleTimeCommand() {
  vk::CommandBufferAllocateInfo allocInfo(stagingCommandPool,
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

  transferQueue.handle.submit(1, &submitInfo, nullptr);
  transferQueue.handle.waitIdle();

  g_device.freeCommandBuffers(stagingCommandPool, 1, &commandbuffer);
}

static void InitCommandbufferAsync() {
  vk::CommandBufferAllocateInfo allocInfo(stagingCommandPool,
                                          vk::CommandBufferLevel::ePrimary, 1);

  for (size_t i = 0; i < commandbuffersAsync.size(); ++i) {
    g_device.allocateCommandBuffers(&allocInfo, &commandbuffersAsync[i]);

    vk::FenceCreateInfo info{vk::FenceCreateFlagBits::eSignaled};
    g_device.createFence(&info, g_allocationCallbacks, &cbAsyncFences[i]);
  }
}

static void SubmitAsyncCommandBegin() {
  vk::CommandBufferBeginInfo beginInfo(
      vk::CommandBufferUsageFlagBits::eSimultaneousUse);

  lastCommandbufferUsedID =
      (lastCommandbufferUsedID + 1) % (commandbuffersAsync.size());

  g_device.waitForFences(1, &cbAsyncFences[lastCommandbufferUsedID], true,
                         std::numeric_limits<uint64_t>::max());
  g_device.resetFences(1, &cbAsyncFences[lastCommandbufferUsedID]);

  commandbuffersAsync[lastCommandbufferUsedID].begin(&beginInfo);
}

static void SubmitAsyncCommandEnd() {

  commandbuffersAsync[lastCommandbufferUsedID].end();

  vk::SubmitInfo submitInfo = {};
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandbuffersAsync[lastCommandbufferUsedID];

  transferQueue.handle.submit(1, &submitInfo,
                              cbAsyncFences[lastCommandbufferUsedID]);
}

namespace Allocator {
void Init() {
  assert(transferQueue.isValid());

  VmaAllocatorCreateInfo allocatorInfo = {};
  allocatorInfo.physicalDevice = g_physicalDevice;
  allocatorInfo.device = g_device;

  vmaCreateAllocator(&allocatorInfo, &vmaAllocator);

  vk::CommandPoolCreateInfo stagingPoolInfo(
      vk::CommandPoolCreateFlagBits::eTransient |
          vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
      transferQueue.index);

  CHECK_VK_RESULT_FATAL(g_device.createCommandPool(&stagingPoolInfo,
                                                   g_allocationCallbacks,
                                                   &stagingCommandPool),
                        "Failed to create staging command pool.");

  InitCommandbufferAsync();
};

void SetTransferQueue(::Queue queue) { transferQueue = queue; }

void WaitForTransferQueue() { transferQueue.handle.waitIdle(); }

void Destroy() {
  vmaDestroyAllocator(vmaAllocator);
  g_device.destroyCommandPool(stagingCommandPool, g_allocationCallbacks);

  for (size_t i = 0; i < cbAsyncFences.size(); ++i)
    g_device.destroyFence(cbAsyncFences[i], g_allocationCallbacks);
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
  if (handle) {
    vmaDestroyBuffer(vmaAllocator, handle, allocation);
    handle = nullptr;
  }
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

  CHECK_VK_RESULT_FATAL(vmaCreateBuffer(vmaAllocator, &bufferInfo, &allocInfo,
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
  vmaMapMemory(vmaAllocator, allocation, &mappedMemory);
  memcpy((char *)mappedMemory + offset, data, (size_t)writeSize);
  vmaUnmapMemory(vmaAllocator, allocation);
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
  case vk::Format::eD32Sfloat:
  case vk::Format::eR8G8B8A8Snorm:
  case vk::Format::eR8G8B8A8Srgb:
  case vk::Format::eR32Sfloat:
    return 4;

  case vk::Format::eR32G32B32A32Sfloat:
    return 16;

  default:
    printf("[WARNING] Unsupported image format specified (%s).\n",
           vk::to_string(format).c_str());
    return 0;
  }
}

Image::Image(Image &&other) {
  handle = other.handle;
  other.handle = nullptr;
  view = other.view;
  other.view = nullptr;

  allocation = std::move(other.allocation);
  size = other.size;

  aspect = other.aspect;
}

Image::~Image() {
  if (view)
    g_device.destroyImageView(view, g_allocationCallbacks);
  if (handle)
    vmaDestroyImage(vmaAllocator, handle, allocation);
}

void Image::allocate(uint32_t texWidth, uint32_t texHeight, vk::Format format,
                     vk::ImageTiling tiling, vk::ImageUsageFlags usage,
                     vk::MemoryPropertyFlags properties,
                     vk::ImageAspectFlags aspectFlags) {
#ifndef NDEBUG
  if (handle)
    printf("[WARNING] Allocating already allocated image. This probably "
           "means memory leaks. (Address = %p)\n",
           (VkImage)handle);
#endif

  size = texWidth * texHeight * GetPixelSizeFromFormat(format);
  aspect = aspectFlags;

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
      vmaCreateImage(vmaAllocator, (VkImageCreateInfo *)&imageInfo, &allocInfo,
                     (VkImage *)&handle, &allocation, nullptr),
      "Failed to create image.");

  vk::ImageViewCreateInfo viewInfo;
  viewInfo.format = format;
  viewInfo.components.r = vk::ComponentSwizzle::eIdentity;
  viewInfo.components.g = vk::ComponentSwizzle::eIdentity;
  viewInfo.components.b = vk::ComponentSwizzle::eIdentity;
  viewInfo.components.a = vk::ComponentSwizzle::eIdentity;
  viewInfo.image = handle;
  viewInfo.viewType = vk::ImageViewType::e2D;
  viewInfo.subresourceRange.aspectMask = aspect;
  viewInfo.subresourceRange.baseArrayLayer = 0;
  viewInfo.subresourceRange.baseMipLevel = 0;
  viewInfo.subresourceRange.layerCount = 1;
  viewInfo.subresourceRange.levelCount = 1;

  view = g_device.createImageView(viewInfo, g_allocationCallbacks);
}

void Image::free() {
  if (view)
    g_device.destroyImageView(view, g_allocationCallbacks);
  if (handle)
    vmaDestroyImage(vmaAllocator, handle, allocation);

  view = nullptr;
  handle = nullptr;

  allocation = VmaAllocation{};
  size = 0;

  aspect = vk::ImageAspectFlags{};
}

void Image::transition_layout(vk::ImageLayout oldLayout,
                              vk::ImageLayout newLayout) {

  SubmitAsyncCommandBegin();

  vk::ImageSubresourceRange subresourceRange;
  subresourceRange.aspectMask = aspect;
  subresourceRange.baseMipLevel = 0;
  subresourceRange.levelCount = 1;
  subresourceRange.baseArrayLayer = 0;
  subresourceRange.layerCount = 1;

  vk::ImageMemoryBarrier barrier;
  barrier.oldLayout = oldLayout;
  barrier.newLayout = newLayout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = handle;
  barrier.subresourceRange = subresourceRange;

  vk::PipelineStageFlags srcStage;
  vk::PipelineStageFlags dstStage;

  if (oldLayout == vk::ImageLayout::eUndefined &&
      newLayout == vk::ImageLayout::eTransferDstOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits(0);
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;

    srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
    dstStage = vk::PipelineStageFlagBits::eTransfer;

  } else if (oldLayout == vk::ImageLayout::eTransferDstOptimal &&
             newLayout == vk::ImageLayout::eShaderReadOnlyOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;

    srcStage = vk::PipelineStageFlagBits::eTransfer;
    dstStage = vk::PipelineStageFlagBits::eFragmentShader;

  } else if (oldLayout == vk::ImageLayout::eUndefined &&
             newLayout ==
                 vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits(0);
    barrier.dstAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentWrite;

    srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
    dstStage = vk::PipelineStageFlagBits::eEarlyFragmentTests;

  } else if (oldLayout ==
                 vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal &&
             newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentRead;
    barrier.dstAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentWrite;

    srcStage = vk::PipelineStageFlagBits::eLateFragmentTests;
    dstStage = vk::PipelineStageFlagBits::eEarlyFragmentTests;

  } else if (oldLayout == vk::ImageLayout::eUndefined &&
             newLayout == vk::ImageLayout::eShaderReadOnlyOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits(0);
    barrier.dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;

    srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
    dstStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;

  } else if (oldLayout == vk::ImageLayout::eShaderReadOnlyOptimal &&
             newLayout == vk::ImageLayout::eColorAttachmentOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits::eShaderRead;
    barrier.dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;

    srcStage = vk::PipelineStageFlagBits::eFragmentShader;
    dstStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;

  } else {

    std::string msg = "Unsupported layout transition."
                      " (" +
                      vk::to_string(oldLayout) + " -> " +
                      vk::to_string(newLayout) + ")";

    throw std::invalid_argument(msg.c_str());
  }

  commandbuffersAsync[lastCommandbufferUsedID].pipelineBarrier(
      srcStage, dstStage, vk::DependencyFlagBits::eByRegion, 0, nullptr, 0,
      nullptr, 1, &barrier);

  SubmitAsyncCommandEnd();
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
  imageSubresource.aspectMask = aspect;
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
  view = other.view;
  other.view = nullptr;

  allocation = std::move(other.allocation);
  size = other.size;

  aspect = other.aspect;

  return *this;
}

} // namespace Renderer
