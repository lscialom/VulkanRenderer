#pragma once

#include <array>
#include <map>
#include <vector>

#include "memory.hpp"

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

} // namespace Renderer
