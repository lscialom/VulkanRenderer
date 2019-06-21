#pragma once

#include "descriptors.hpp"

namespace Renderer {
struct Texture {
private:
  Image image;

public:
  bool init(const std::string &path);

  const Image *get_image() const { return &image; }
};
} // namespace Renderer
