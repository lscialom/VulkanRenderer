#pragma once

#include <stdint.h>

struct VkAllocationCallbacks;
typedef struct VkInstance_T *VkInstance;
typedef struct VkSurfaceKHR_T *VkSurfaceKHR;

namespace WindowHandler {
void Init(uint32_t width, uint32_t height);
bool Update();
void Shutdown();

uint32_t GetRequiredInstanceExtensions(const char **&extensions);
int32_t CreateSurface(VkInstance instance,
                      const VkAllocationCallbacks *allocator,
                      VkSurfaceKHR *surface);

void GetFramebufferSize(int *width, int *height);
void SetFramebufferSizeCallbackSignal(bool *callback);
} // namespace WindowHandler
