#include "window_handler.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

namespace WindowHandler
{
	static GLFWwindow* g_window;

	static bool* g_framebufferResizeCallbackSignal = nullptr;
	static void framebufferResizeCallback(GLFWwindow*, int, int)
	{
		if (g_framebufferResizeCallbackSignal == nullptr)
			return;

		*g_framebufferResizeCallbackSignal = true;
	}

	void Init(uint32_t width, uint32_t height)
	{
		glfwInit();

		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

		g_window = glfwCreateWindow(width, height, "Vulkan", nullptr, nullptr);
		glfwSetFramebufferSizeCallback(g_window, framebufferResizeCallback);
	}

	uint32_t GetRequiredInstanceExtensions(const char**& extensions)
	{
		uint32_t glfwExtensionCount = 0;
		extensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

		return glfwExtensionCount;
	}

	int32_t CreateSurface(VkInstance instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* surface)
	{
		return glfwCreateWindowSurface(instance, g_window, allocator, surface);
	}

	void GetFramebufferSize(int* width, int* height)
	{
		glfwGetFramebufferSize(g_window, width, height);
	}

	void SetFramebufferSizeCallbackSignal(bool* callback)
	{
		g_framebufferResizeCallbackSignal = callback;
	}

	bool Update()
	{
		glfwPollEvents();

		return !glfwWindowShouldClose(g_window);
	}

	void Shutdown()
	{
		glfwDestroyWindow(g_window);

		glfwTerminate();
	}
}