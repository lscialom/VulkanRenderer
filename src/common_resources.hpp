#pragma once

#include "maths.hpp"
#include "memory.hpp"
#include "descriptors.hpp"

#include <vulkan/vulkan.hpp>

#include <array>

namespace Renderer
{
	namespace CommonResources
	{
		#include "descriptor_layouts.inl"
		#include "samplers.inl"
		#include "ssao_resources.inl"
	}
}