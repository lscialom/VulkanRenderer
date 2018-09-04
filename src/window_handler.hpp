#pragma once

#include <cstdint>

struct VkInstance_T;
struct VkAllocationCallbacks;
struct VkSurfaceKHR_T;

namespace WindowHandler
{
	using VkInstance = VkInstance_T*;
	using VkSurfaceKHR = VkSurfaceKHR_T*;

	void Init(uint32_t width, uint32_t height);
	bool Update();
	void Shutdown();

	uint32_t GetRequiredInstanceExtensions(const char**& extensions);
	int32_t CreateSurface(VkInstance instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* surface);
}