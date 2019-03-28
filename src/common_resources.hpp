#pragma once

#include "data_structures.hpp"
#include "descriptors.hpp"
#include "maths.hpp"
#include "memory.hpp"

#include <vulkan/vulkan.hpp>

#include <array>

namespace Renderer {
namespace CommonResources {
#include "descriptor_layouts.inl"
#include "uniform_buffer_objects.inl"
#include "samplers.inl"
#include "postprocess_resources.inl"
} // namespace CommonResources
} // namespace Renderer
