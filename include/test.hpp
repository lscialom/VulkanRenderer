#pragma once

#ifdef VULKAN_RENDERER_DLL_EXPORTS
	#define VULKAN_RENDERER_EXPORTS __declspec(dllexport)
#else
	#define VULKAN_RENDERER_EXPORTS __declspec(dllimport)
#endif

VULKAN_RENDERER_EXPORTS int test();