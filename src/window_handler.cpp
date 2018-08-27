#include "window_handler.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

namespace WindowHandler
{
	static GLFWwindow* g_window;

	void Init(unsigned int width, unsigned int height)
	{
		glfwInit();

		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
		glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

		g_window = glfwCreateWindow(width, height, "Vulkan", nullptr, nullptr);
	}

	uint32_t GetRequiredInstanceExtensions(const char**& extensions)
	{
		uint32_t glfwExtensionCount = 0;
		extensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

		return glfwExtensionCount;
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