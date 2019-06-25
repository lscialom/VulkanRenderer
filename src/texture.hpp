#pragma once

#include "descriptors.hpp"

namespace Renderer {
enum class TextureUsage : uint8_t { Color, Data };

struct Texture {
private:
  Image image;

public:
  bool init(const std::string &path, TextureUsage usage);

  const Image *get_image() const { return &image; }
};
} // namespace Renderer
