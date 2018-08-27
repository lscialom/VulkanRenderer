#pragma once

#include "config.hpp"

namespace Renderer
{
	VULKAN_RENDERER_EXPORTS void Init(unsigned int width, unsigned int height);

	VULKAN_RENDERER_EXPORTS bool Update();
	VULKAN_RENDERER_EXPORTS void Shutdown();

	VULKAN_RENDERER_EXPORTS void Run(unsigned int width, unsigned int height);
}