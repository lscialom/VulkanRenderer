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

void SetTransferQueue(::Renderer::Queue queue) { transferQueue = queue; }

void WaitForTransferQueue() { transferQueue.handle.waitIdle(); }

uint8_t GetPixelSizeFromFormat(vk::Format format) {
  switch (format) {
  case vk::Format::eR8G8B8A8Unorm:
  case vk::Format::eD32Sfloat:
  case vk::Format::eR8G8B8A8Snorm:
  case vk::Format::eR8G8B8A8Srgb:
  case vk::Format::eR32Sfloat:
    return 4;

  case vk::Format::eR32G32B32A32Sfloat:
    return 16;

  case vk::Format::eR16G16B16A16Sfloat:
    return 8;

  case vk::Format::eR8Uint:
  case vk::Format::eR8Unorm:
    return 1;

  default:
    printf("[WARNING] Unsupported image format specified (%s).\n",
           vk::to_string(format).c_str());
    return 0;
  }
}

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
  memProperties = properties;

  VkBufferCreateInfo bufferInfo = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  bufferInfo.size = size;
  bufferInfo.usage = (VkBufferUsageFlags)usage;
  bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VmaAllocationCreateInfo allocInfo = {};
  allocInfo.usage = VMA_MEMORY_USAGE_UNKNOWN;
  allocInfo.requiredFlags = (VkMemoryPropertyFlags)memProperties;

  VmaAllocationInfo resultInfo = {};

  CHECK_VK_RESULT_FATAL(vmaCreateBuffer(vmaAllocator, &bufferInfo, &allocInfo,
                                        (VkBuffer *)&handle, &allocation,
                                        &resultInfo),
                        "Failed to create buffer.");
}

// TODO Staging queue (TRANSFER_BIT)
void Buffer::copy_to(const Buffer &dstBuffer, VkDeviceSize copySize,
                     VkDeviceSize dstOffset) const {

  vk::BufferCopy copyRegion(0, // TODO src offset
                            dstOffset, copySize);

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

  // TODO Support other vk::MemoryPropertyFlagBits
  if ((memProperties & vk::MemoryPropertyFlagBits::eDeviceLocal) !=
      vk::MemoryPropertyFlagBits::eDeviceLocal) {

    void *mappedMemory;

    vmaMapMemory(vmaAllocator, allocation, &mappedMemory);
    memcpy((char *)mappedMemory + offset, data, (size_t)writeSize);
    vmaUnmapMemory(vmaAllocator, allocation);

  } else {

    Buffer stagingBuffer;
    stagingBuffer.allocate(writeSize, vk::BufferUsageFlagBits::eTransferSrc,
                           vk::MemoryPropertyFlagBits::eHostVisible |
                               vk::MemoryPropertyFlagBits::eHostCoherent);

    stagingBuffer.write(data, writeSize);

    stagingBuffer.copy_to(*this, writeSize, offset);
  }
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

void Image::allocate(uint32_t texWidth, uint32_t texHeight,
                     vk::Format texFormat, vk::ImageTiling tiling,
                     vk::ImageUsageFlags usage,
                     vk::MemoryPropertyFlags properties, uint32_t p_mipLevels,
                     vk::ImageAspectFlags aspectFlags) {
#ifndef NDEBUG
  if (handle)
    printf("[WARNING] Allocating already allocated image. This probably "
           "means memory leaks. (Address = %p)\n",
           (VkImage)handle);
#endif

  width = texWidth;
  height = texHeight;

  aspect = aspectFlags;
  format = texFormat;
  mipLevels = p_mipLevels;

  size = width * height * Allocator::GetPixelSizeFromFormat(format);

  vk::ImageCreateInfo imageInfo{
      vk::ImageCreateFlags(),
      vk::ImageType::e2D,
      format,
      {texWidth, texHeight, 1},
      mipLevels,
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
  viewInfo.subresourceRange.levelCount = mipLevels;

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
  subresourceRange.levelCount = mipLevels;
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

  } else if (oldLayout == vk::ImageLayout::eTransferDstOptimal &&
             newLayout == vk::ImageLayout::eTransferDstOptimal) {

    barrier.srcAccessMask = vk::AccessFlagBits::eTransferRead;
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;

    srcStage = vk::PipelineStageFlagBits::eTransfer;
    dstStage = vk::PipelineStageFlagBits::eTransfer;

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
void Image::write_from_buffer(vk::Buffer buffer, uint32_t dimX, uint32_t dimY) {
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
  region.imageExtent = vk::Extent3D(dimX, dimY, 1);

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

void Image::generateMipMaps() {

  vk::CommandBuffer commandBuffer = BeginSingleTimeCommand();

  vk::ImageMemoryBarrier barrier{};
  barrier.image = handle;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = 1;
  barrier.subresourceRange.levelCount = 1;

  int32_t mipWidth = width;
  int32_t mipHeight = height;

  for (uint32_t i = 1; i < mipLevels; i++) {

    barrier.subresourceRange.baseMipLevel = i - 1;
    barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal;
    barrier.newLayout = vk::ImageLayout::eTransferSrcOptimal;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferRead;

    commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                  vk::PipelineStageFlagBits::eTransfer,
                                  vk::DependencyFlagBits(0), 0, nullptr, 0,
                                  nullptr, 1, &barrier);

    vk::ImageBlit blit{};
    blit.srcOffsets[0] = vk::Offset3D{0, 0, 0};
    blit.srcOffsets[1] = vk::Offset3D{mipWidth, mipHeight, 1};
    blit.srcSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
    blit.srcSubresource.mipLevel = i - 1;
    blit.srcSubresource.baseArrayLayer = 0;
    blit.srcSubresource.layerCount = 1;
    blit.dstOffsets[0] = vk::Offset3D{0, 0, 0};
    blit.dstOffsets[1] = vk::Offset3D{mipWidth > 1 ? mipWidth / 2 : 1,
                                      mipHeight > 1 ? mipHeight / 2 : 1, 1};
    blit.dstSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
    blit.dstSubresource.mipLevel = i;
    blit.dstSubresource.baseArrayLayer = 0;
    blit.dstSubresource.layerCount = 1;

    commandBuffer.blitImage(handle, vk::ImageLayout::eTransferSrcOptimal,
                            handle, vk::ImageLayout::eTransferDstOptimal, 1,
                            &blit, vk::Filter::eLinear);

    barrier.oldLayout = vk::ImageLayout::eTransferSrcOptimal;
    barrier.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferRead;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;

    commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                  vk::PipelineStageFlagBits::eFragmentShader,
                                  vk::DependencyFlagBits(0), 0, nullptr, 0,
                                  nullptr, 1, &barrier);

    if (mipWidth > 1)
      mipWidth /= 2;
    if (mipHeight > 1)
      mipHeight /= 2;
  }

  barrier.subresourceRange.baseMipLevel = mipLevels - 1;
  barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal;
  barrier.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
  barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
  barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;

  commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                vk::PipelineStageFlagBits::eFragmentShader,
                                vk::DependencyFlagBits(0), 0, nullptr, 0,
                                nullptr, 1, &barrier);

  EndSingleTimeCommand(commandBuffer);
}

} // namespace Renderer
