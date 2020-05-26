#pragma once

#include "descriptors.hpp"

namespace Renderer {
enum class TextureUsage : uint8_t { Color, Data };

struct Texture {
private:
  Image image;
  uint32_t mipLevels = 1;

public:
  bool init(const std::string &path, TextureUsage usage);

  const Image *get_image() const { return &image; }
  uint32_t get_mipLevels() const { return mipLevels; }
};
} // namespace Renderer
