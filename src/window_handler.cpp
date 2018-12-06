#include "window_handler.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

namespace WindowHandler {
static GLFWwindow *window;

static bool *framebufferResizeCallbackSignal = nullptr;
static void framebufferResizeCallback(GLFWwindow *, int, int) {
  if (framebufferResizeCallbackSignal == nullptr)
    return;

  *framebufferResizeCallbackSignal = true;
}

void Init(uint32_t width, uint32_t height) {
  glfwInit();

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  window = glfwCreateWindow(width, height, "Vulkan", nullptr, nullptr);
  glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);
}

uint32_t GetRequiredInstanceExtensions(const char **&extensions) {
  uint32_t glfwExtensionCount = 0;
  extensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

  return glfwExtensionCount;
}

int32_t CreateSurface(VkInstance instance,
                      const VkAllocationCallbacks *allocator,
                      VkSurfaceKHR *surface) {
  return glfwCreateWindowSurface(instance, window, allocator, surface);
}

void GetFramebufferSize(int *width, int *height) {
  glfwGetFramebufferSize(window, width, height);
}

void SetFramebufferSizeCallbackSignal(bool *callback) {
  framebufferResizeCallbackSignal = callback;
}

bool Update() {
  glfwPollEvents();

  return !glfwWindowShouldClose(window);
}

void Shutdown() {
  glfwDestroyWindow(window);

  glfwTerminate();
}
} // namespace WindowHandler
