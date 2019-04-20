#pragma once

#include <array>
#include <map>
#include <vector>

#include "global_context.hpp"
#include "configuration_helper.hpp"
#include "memory.hpp"
#include "swapchain.hpp"

namespace Renderer {

struct DescriptorLayout {
  vk::DescriptorSetLayout handle;
  std::vector<vk::DescriptorSetLayoutBinding> bindings;
};

struct DescriptorSetInfo {
  DescriptorLayout layout;
  std::vector<const Buffer *> buffers;
  std::vector<const Image *> images;
  std::vector<const vk::Sampler *> samplers;
};

struct DescriptorSet {
private:
  // TODO Optimize
  vk::DescriptorPool pool;
  vk::DescriptorSet set;

  // Does not own it
  vk::DescriptorSetLayout layout;

  std::vector<vk::DescriptorSetLayoutBinding> bindingsInfo;

  std::vector<const Buffer *> buffers;
  std::vector<const Image *> images;
  std::vector<const vk::Sampler *> samplers;

public:
  ~DescriptorSet() { destroy(); }

  void init(const DescriptorSetInfo &info, bool updateNow = true);

  void update() const;

  void destroy();

  const vk::DescriptorSet &get_handle() const { return set; }
  vk::DescriptorSetLayout get_layout() const { return layout; }
};

// TODO Find a way to move UniformObjectLayout here

struct UniformBufferObject {
private:
  std::vector<Buffer> buffers;
  std::vector<vk::DescriptorSet> descriptorSets;

  vk::DescriptorPool pool;

  // Does not own it
  vk::DescriptorSetLayout layout;

  size_t alignment = 0;

public:
  UniformBufferObject() = default;
  UniformBufferObject(UniformBufferObject &&other) {
    descriptorSets.resize(other.descriptorSets.size());
    for (size_t i = 0; i < other.descriptorSets.size(); ++i) {
      descriptorSets[i] = other.descriptorSets[i];
      other.descriptorSets[i] = nullptr;
    }

    pool = other.pool;
    other.pool = nullptr;

    buffers = std::move(other.buffers);
    alignment = other.alignment;
  }
  ~UniformBufferObject() {
    if (pool) {
      g_device.destroyDescriptorPool(pool, g_allocationCallbacks);
      pool = nullptr;
    }
  }

  size_t get_alignment() const { return alignment; }

  const vk::DescriptorSet &get_descriptor_set(size_t index) const {
    return descriptorSets[index];
  }
  vk::DescriptorSetLayout get_layout() const { return layout; }

  template <typename T> void init(T layoutInfo) {
    if (T::DescriptorType == vk::DescriptorType::eUniformBufferDynamic ||
        T::DescriptorType == vk::DescriptorType::eUniformBuffer) {
      size_t minUboAlignment = g_physicalDevice.getProperties()
                                   .limits.minUniformBufferOffsetAlignment;
      alignment = T::Size;

      if (minUboAlignment > 0) {
        alignment = (alignment + minUboAlignment - 1) & ~(minUboAlignment - 1);
      }
    } else
      alignment = T::Size;

    layout = layoutInfo.layout;

    size_t nbElements = T::ArraySize;
    size_t nbBuffers = Swapchain::ImageCount();

    vk::DescriptorPoolSize poolSize{vk::DescriptorType::eUniformBuffer,
                                    static_cast<uint32_t>(nbBuffers)};

    vk::DescriptorPoolCreateInfo poolInfo{vk::DescriptorPoolCreateFlags(),
                                          static_cast<uint32_t>(nbBuffers), 1,
                                          &poolSize};

    CHECK_VK_RESULT_FATAL(
        g_device.createDescriptorPool(&poolInfo, g_allocationCallbacks, &pool),
        "Failed to create descriptor pool.");

    buffers.resize(nbBuffers);
    for (size_t i = 0; i < buffers.size(); ++i)
      buffers[i].allocate(alignment * nbElements,
                          vk::BufferUsageFlagBits::eUniformBuffer,
                          vk::MemoryPropertyFlagBits::eHostCoherent |
                              vk::MemoryPropertyFlagBits::eHostVisible);

    std::vector<vk::DescriptorSetLayout> layouts(nbBuffers, layout);

    vk::DescriptorSetAllocateInfo allocInfo{
        pool, static_cast<uint32_t>(nbBuffers), layouts.data()};

    descriptorSets.resize(nbBuffers);
    CHECK_VK_RESULT_FATAL(
        g_device.allocateDescriptorSets(&allocInfo, descriptorSets.data()),
        "Failed to allocate descriptor sets.");

    for (size_t i = 0; i < descriptorSets.size(); ++i) {

      std::vector<vk::DescriptorBufferInfo> bInfos;
      if (T::DescriptorType == vk::DescriptorType::eUniformBufferDynamic) {

        vk::DescriptorBufferInfo bufferInfo{buffers[i].get_handle(), 0,
                                            alignment * nbElements};
        bInfos.push_back(bufferInfo);

      } else {
        for (size_t j = 0; j < nbElements; ++j) {
          vk::DescriptorBufferInfo bufferInfo{buffers[i].get_handle(),
                                              alignment * j, alignment};
          bInfos.push_back(bufferInfo);
        }
      }

      vk::WriteDescriptorSet descriptorWrite{descriptorSets[i],
                                             0,
                                             0,
                                             (uint32_t)bInfos.size(),
                                             T::DescriptorType,
                                             nullptr,
                                             bInfos.data(),
                                             nullptr};

      g_device.updateDescriptorSets(1, &descriptorWrite, 0, nullptr);
    }
  }

  void write(uint32_t bufferIndex, const void *data, VkDeviceSize writeSize,
             uint64_t step) const {

    // TODO Support alignment for multiple elements at once
    buffers[bufferIndex].write(data, writeSize, step * alignment);
  }
};

} // namespace Renderer
