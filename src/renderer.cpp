#include "renderer.hpp"
#include "window_handler.hpp"

#include <vulkan/vulkan.hpp>

#include <iostream>
#include <vector>

#define CHECK_VK_RESULT_FATAL(vk_function, msg) { \
	vk::Result res; \
	if ((res = vk_function) != vk::Result::eSuccess) \
	{ \
		std::string err = "[FATAL]"; err += msg; err += " : "; err += vk::to_string(res); \
		throw std::runtime_error(err.c_str()); \
	} \
}

#define TRY_CATCH_BLOCK(msg, code) try \
{ \
	code \
} \
catch (const std::exception& e) \
{ \
	std::cerr << e.what() << "\n" << "[FATAL]" << msg << std::endl; \
	abort(); \
}

namespace Renderer
{
	static vk::Instance g_instance;
	static vk::AllocationCallbacks* g_allocator = nullptr;

	static void CreateInstance()
	{
		vk::ApplicationInfo appInfo(
			"Renderer",
		    VK_MAKE_VERSION(1, 0, 0),
		    "No Engine",
		    VK_MAKE_VERSION(1, 0, 0),
		    VK_API_VERSION_1_1
		);

		vk::InstanceCreateInfo createInfo;
		createInfo.pApplicationInfo = &appInfo;

		const char** pExtensions = nullptr;

		#ifndef NDEBUG
		const std::vector<const char*> validationLayers = {
			"VK_LAYER_LUNARG_standard_validation",
			"VK_LAYER_LUNARG_monitor"
		};

		createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		createInfo.ppEnabledLayerNames = validationLayers.data();

		uint32_t requiredExtensionsCount = WindowHandler::GetRequiredInstanceExtensions(pExtensions);

		std::vector<const char*> extensions(pExtensions, pExtensions + requiredExtensionsCount);
		extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

		createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
		createInfo.ppEnabledExtensionNames = extensions.data();
		#else
		createInfo.enabledLayerCount = 0;
		createInfo.enabledExtensionCount = WindowHandler::GetRequiredInstanceExtensions(pExtensions);
		createInfo.ppEnabledExtensionNames = pExtensions;
		#endif

		CHECK_VK_RESULT_FATAL(vk::createInstance(&createInfo, g_allocator, &g_instance), "Failed to init vulkan instance")
	}

	static void InitVulkan()
	{
		CreateInstance();
	}

	void Init(unsigned int width, unsigned int height)
	{
		TRY_CATCH_BLOCK("Failed to init renderer",

			WindowHandler::Init(width, height);
			InitVulkan();
		);
	}

	bool Update()
	{
		TRY_CATCH_BLOCK("Failed to update renderer",

			return WindowHandler::Update();
		);
	}

	void Shutdown()
	{
		TRY_CATCH_BLOCK("Failed to shutdown renderer",

			g_instance.destroy(g_allocator);
			WindowHandler::Shutdown();
		);
	}

	void Run(unsigned int width, unsigned int height)
	{
		Init(width, height);
		while (Update());
		Shutdown();
	}


}