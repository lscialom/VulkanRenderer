#include "descriptors.hpp"

namespace Renderer {
struct Texture {
private:
  DescriptorSet descriptorSet;
  Image image;

public:
  bool init(const std::string &path);

  const vk::DescriptorSet *get_descriptor_set() const {
    return &descriptorSet.get_handle();
  }
};
} // namespace Renderer
