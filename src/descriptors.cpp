#include "descriptors.hpp"

#include "debug_tools.hpp"
#include "global_context.hpp"

#include <vulkan/vulkan.hpp>

namespace Renderer {

void DescriptorSet::init(const DescriptorSetInfo &info, bool updateNow) {

  bindingsInfo.resize(info.layout.bindings.size());

  memcpy(bindingsInfo.data(), info.layout.bindings.data(),
         info.layout.bindings.size() * sizeof(VkDescriptorSetLayoutBinding));

  layout = info.layout.handle;

  buffers = info.buffers;
  images = info.images;
  samplers = info.samplers;

  std::map<vk::DescriptorType, uint32_t> poolSizesInfo;

  for (const auto &binding : info.layout.bindings)
    poolSizesInfo[(vk::DescriptorType)binding.descriptorType] +=
        binding.descriptorCount;

  std::vector<vk::DescriptorPoolSize> poolSizes;
  for (const auto &info : poolSizesInfo) {
    vk::DescriptorPoolSize poolSize;
    poolSize.type = info.first;
    poolSize.descriptorCount = info.second;

    poolSizes.push_back(poolSize);
  }

  vk::DescriptorPoolCreateInfo poolInfo;
  poolInfo.maxSets = 1;
  poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
  poolInfo.pPoolSizes = poolSizes.data();

  CHECK_VK_RESULT_FATAL(
      g_device.createDescriptorPool(&poolInfo, g_allocationCallbacks, &pool),
      "Failed to create descriptor pool.");

  vk::DescriptorSetAllocateInfo allocInfo;
  allocInfo.descriptorPool = pool;
  allocInfo.descriptorSetCount = 1;
  allocInfo.pSetLayouts = &layout;

  g_device.allocateDescriptorSets(&allocInfo, &set);

  if (updateNow)
    update();
}

void DescriptorSet::update() const {

  std::vector<vk::WriteDescriptorSet> writes;
  uint32_t currentBufferIndex = 0;
  uint32_t currentImageIndex = 0;

  std::vector<vk::DescriptorBufferInfo *> bufferInfosToDestroy;
  std::vector<vk::DescriptorImageInfo *> imageInfosToDestroy;

  for (size_t i = 0; i < bindingsInfo.size(); ++i) {

    std::string descriptorTypeStr =
        vk::to_string(bindingsInfo[i].descriptorType);

    if (descriptorTypeStr.find("Texel") != std::string::npos) {

      // TODO Implement Texel Buffer
      std::string err = "Unmanaged descriptor type used (";
      err += descriptorTypeStr;
      err += ")";
      throw std::runtime_error(err.c_str());

    } else if (descriptorTypeStr.find("Buffer") != std::string::npos) {

      vk::DescriptorBufferInfo *infos =
          new vk::DescriptorBufferInfo[bindingsInfo[i].descriptorCount];
      bufferInfosToDestroy.push_back(infos);

      for (uint32_t j = 0; j < bindingsInfo[i].descriptorCount; ++j) {
        infos[j].buffer = buffers[currentBufferIndex]->get_handle();
        infos[j].offset = 0; // TODO manage offsets
        infos[j].range = buffers[currentBufferIndex]->get_size();

        currentBufferIndex++;
      }

      vk::WriteDescriptorSet writeSet;
      writeSet.descriptorCount = bindingsInfo[i].descriptorCount;
      writeSet.descriptorType = bindingsInfo[i].descriptorType;
      writeSet.dstArrayElement = 0; // TODO check that it works for every type
      writeSet.dstBinding = bindingsInfo[i].binding;
      writeSet.dstSet = set;
      writeSet.pBufferInfo = infos;

      writes.push_back(writeSet);

    } else if (descriptorTypeStr.find("Image") != std::string::npos) {

      vk::DescriptorImageInfo *infos =
          new vk::DescriptorImageInfo[bindingsInfo[i].descriptorCount];
      imageInfosToDestroy.push_back(infos);

      for (uint32_t j = 0; j < bindingsInfo[i].descriptorCount; ++j) {

        if ((images[currentImageIndex]->get_aspect() &
             vk::ImageAspectFlagBits::eColor) ==
            vk::ImageAspectFlagBits::eColor) {

          infos[j].imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;

        } else if ((images[currentImageIndex]->get_aspect() &
                    vk::ImageAspectFlagBits::eDepth) ==
                   vk::ImageAspectFlagBits::eDepth) {
          infos[j].imageLayout =
              vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal;

        } else {

          // TODO Support other
          // layouts

          std::string err = "Unmanaged Image Aspect used in descriptor. (";
          err += vk::to_string(images[currentImageIndex]->get_aspect());
          err += ")";
          throw std::runtime_error(err.c_str());
        }

        infos[j].imageView = images[currentImageIndex]->get_view();
        infos[j].sampler = *samplers[currentImageIndex];

        currentImageIndex++;
      }

      vk::WriteDescriptorSet writeSet;
      writeSet.descriptorCount = bindingsInfo[i].descriptorCount;
      writeSet.descriptorType = bindingsInfo[i].descriptorType;
      writeSet.dstArrayElement = 0; // TODO check that it works for every type
      writeSet.dstBinding = bindingsInfo[i].binding;
      writeSet.dstSet = set;
      writeSet.pImageInfo = infos;

      writes.push_back(writeSet);

    } else {

      // TODO Implement Input Attachment and other potential new types
      std::string err = "Unmanaged descriptor type used (";
      err += descriptorTypeStr;
      err += ")";
      throw std::runtime_error(err.c_str());
    }
  }

  g_device.updateDescriptorSets(static_cast<uint32_t>(writes.size()),
                                writes.data(), 0, nullptr);

  for (size_t i = 0; i < bufferInfosToDestroy.size(); ++i)
    delete bufferInfosToDestroy[i];

  for (size_t i = 0; i < imageInfosToDestroy.size(); ++i)
    delete imageInfosToDestroy[i];
}

void DescriptorSet::destroy() {
  if (pool) {
    g_device.destroyDescriptorPool(pool, g_allocationCallbacks);
    pool = nullptr;
  }
}

} // namespace Renderer
