#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "texture.hpp"

#include "common_resources.hpp"

#include <string>

namespace Renderer {

bool Texture::init(const std::string &path, TextureUsage usage) {
  int texWidth, texHeight, texChannels;
  stbi_uc *pixels =
      stbi_load(path.c_str(), &texWidth, &texHeight, &texChannels,
                STBI_rgb_alpha); // TODO Support multiple pixel formats

  if (!pixels) {
    printf("[Error] Failed to load texture image %s\n", path.c_str());
    return false;
  }

  vk::Format imageFormat;

  switch (usage) {
  case TextureUsage::Color:
    imageFormat = vk::Format::eR8G8B8A8Srgb;
    mipLevels = static_cast<uint32_t>(
                    std::floor(std::log2(std::max(texWidth, texHeight)))) +
                1;

    image.allocate(texWidth, texHeight, imageFormat, vk::ImageTiling::eOptimal,
                   vk::ImageUsageFlagBits::eTransferSrc |
                       vk::ImageUsageFlagBits::eTransferDst |
                       vk::ImageUsageFlagBits::eSampled,
                   vk::MemoryPropertyFlagBits::eDeviceLocal, mipLevels);

    // cast for correct template deduction (for inner staging buffer size
    // deduction)
    // TODO Support multiple image formats
    image.write_from_raw_data(reinterpret_cast<uint32_t *>(pixels),
                              vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eTransferDstOptimal);
    image.generateMipMaps();
    break;

    // TODO eR8G8B8A8Snorm for normal maps ?

  case TextureUsage::Data:
  default:
    imageFormat = vk::Format::eR8G8B8A8Unorm;
    image.allocate(texWidth, texHeight, imageFormat, vk::ImageTiling::eOptimal,
                   vk::ImageUsageFlagBits::eTransferDst |
                       vk::ImageUsageFlagBits::eSampled,
                   vk::MemoryPropertyFlagBits::eDeviceLocal, mipLevels);

    // cast for correct template deduction (for inner staging buffer size
    // deduction)
    // TODO Support multiple image formats
    image.write_from_raw_data(reinterpret_cast<uint32_t *>(pixels),
                              vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eShaderReadOnlyOptimal);
    break;
  }

  // DescriptorSetInfo descriptorSetInfo;
  // descriptorSetInfo.layout = CommonResources::UniqueTextureLayout;
  // descriptorSetInfo.images = {&image};
  // descriptorSetInfo.samplers = {&CommonResources::TextureSampler};

  // descriptorSet.init(descriptorSetInfo);

  return true;
}
} // namespace Renderer
