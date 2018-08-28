#include "renderer.hpp"
#include "window_handler.hpp"

#include <vulkan/vulkan.hpp>

#include <iostream>

#include <vector>
#include <map>

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
	static vk::PhysicalDevice g_physicalDevice = nullptr;

	static vk::Device g_device = nullptr;

	struct Queue
	{
		int index = -1;
		vk::Queue queue = nullptr;
	};

	static Queue g_graphicsQueue;

	static vk::AllocationCallbacks* g_allocator = nullptr;

	static vk::DispatchLoaderDynamic g_dldy;

	static std::vector<const char*> g_validationLayers;

	#ifndef NDEBUG
	static vk::DebugUtilsMessengerEXT g_debugMessenger;

	static VKAPI_ATTR VkBool32 VKAPI_CALL g_debugCallback(
		VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
		VkDebugUtilsMessageTypeFlagsEXT messageType,
		const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
		void* userData)
	{
		char prefix[64];
		char *message = (char *)malloc(strlen(callbackData->pMessage) + 500);
		assert(message);
		if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)
		{
			strcpy(prefix, "VERBOSE : ");
		}
		else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT)
		{
			strcpy(prefix, "INFO : ");
		}
		else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
		{
			strcpy(prefix, "WARNING : ");
		}
		else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
		{
			strcpy(prefix, "ERROR : ");
		}
		if (messageType & VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT)
		{
			strcat(prefix, "GENERAL");
		}
		else
		{
			if (messageType & VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT)
			{
				strcat(prefix, "SPEC");
			}
			if (messageType & VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT)
			{
				if (messageType & VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT)
				{
					strcat(prefix, "|");
				}
				strcat(prefix, "PERF");
			}
		}

		sprintf(message,
			"%s - Message ID Number %d, Message ID Name : %s\n\t%s",
			prefix,
			callbackData->messageIdNumber,
			callbackData->pMessageIdName,
			callbackData->pMessage);
		if (callbackData->objectCount > 0)
		{
			char tmp_message[500];
			sprintf(tmp_message, "\n\n\t Objects - %d\n", callbackData->objectCount);
			strcat(message, tmp_message);
			for (uint32_t object = 0; object < callbackData->objectCount; ++object)
			{
				sprintf(tmp_message,
					"\t\t Object[%d] - Type %s, Value %p, Name \"%s\"\n",
					object,
					vk::to_string((vk::ObjectType)callbackData->pObjects[object].objectType).c_str(),
					(void*)(callbackData->pObjects[object].objectHandle),
					callbackData->pObjects[object].pObjectName);
				strcat(message, tmp_message);
			}
		}
		if (callbackData->cmdBufLabelCount > 0)
		{
			char tmp_message[500];
			sprintf(tmp_message,
				"\n\n\t Command Buffer Labels - %d\n",
				callbackData->cmdBufLabelCount);
			strcat(message, tmp_message);
			for (uint32_t label = 0; label < callbackData->cmdBufLabelCount; ++label)
			{
				sprintf(tmp_message,
					"\t\t Label[%d] - %s { %f, %f, %f, %f}\n",
					label,
					callbackData->pCmdBufLabels[label].pLabelName,
					callbackData->pCmdBufLabels[label].color[0],
					callbackData->pCmdBufLabels[label].color[1],
					callbackData->pCmdBufLabels[label].color[2],
					callbackData->pCmdBufLabels[label].color[3]);
				strcat(message, tmp_message);
			}
		}
		printf("%s\n", message);
		fflush(stdout);
		free(message);

		return VK_FALSE;
	}
	#endif

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
		g_validationLayers = {
			"VK_LAYER_LUNARG_standard_validation",
			"VK_LAYER_LUNARG_monitor"
		};

		createInfo.enabledLayerCount = static_cast<uint32_t>(g_validationLayers.size());
		createInfo.ppEnabledLayerNames = g_validationLayers.data();

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

		CHECK_VK_RESULT_FATAL(vk::createInstance(&createInfo, g_allocator, &g_instance), "Failed to init vulkan instance");

		g_dldy.init(g_instance);

		#ifndef NDEBUG
		vk::DebugUtilsMessengerCreateInfoEXT dbgCreateInfo(
			vk::DebugUtilsMessengerCreateFlagsEXT(),
			vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose | vk::DebugUtilsMessageSeverityFlagBitsEXT::eInfo | vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning | vk::DebugUtilsMessageSeverityFlagBitsEXT::eError,
			vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral | vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation | vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance,
			g_debugCallback
		);

		g_debugMessenger = g_instance.createDebugUtilsMessengerEXT(dbgCreateInfo, g_allocator, g_dldy);
		#endif
	}

	static int GetAvailableQueueFamily(vk::PhysicalDevice device, vk::QueueFlags capabilities)
	{
		std::vector<vk::QueueFamilyProperties> queueFamilies = device.getQueueFamilyProperties();
		int i = 0;
		for (const auto& queueFamily : queueFamilies)
		{
			if (queueFamily.queueCount > 0 && queueFamily.queueFlags & capabilities)
				return i;

			i++;
		}
	}

	static int RateDeviceSuitability(vk::PhysicalDevice device, vk::QueueFlags capabilities)
	{
		if (GetAvailableQueueFamily(device, capabilities) == -1)
			return 0;

		vk::PhysicalDeviceProperties deviceProperties = device.getProperties();
		//vk::PhysicalDeviceFeatures deviceFeatures = device.getFeatures();

		int score = 0;

		// Discrete GPUs have a significant performance advantage
		if (deviceProperties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu)
			score += 1000;

		// Maximum possible size of textures affects graphics quality
		score += deviceProperties.limits.maxImageDimension2D;

		return score;
	}

	static void InitDevice()
	{
		uint32_t deviceCount = 0;
		vkEnumeratePhysicalDevices(g_instance, &deviceCount, nullptr);

		if (deviceCount == 0)
			throw std::runtime_error("failed to find GPUs with Vulkan support!");

		std::vector<vk::PhysicalDevice> devices(deviceCount);
		g_instance.enumeratePhysicalDevices(&deviceCount, devices.data());

		vk::QueueFlags desiredQueueCapabilities = vk::QueueFlagBits::eGraphics;

		{
			// Use an ordered map to automatically sort candidates by increasing score
			std::multimap<int, vk::PhysicalDevice> candidates;

			for (const auto& device : devices)
			{
				int score = RateDeviceSuitability(device, desiredQueueCapabilities);
				candidates.insert(std::make_pair(score, device));
			}

			// Check if the best candidate is suitable at all
			if (candidates.rbegin()->first > 0)
				g_physicalDevice = candidates.rbegin()->second;
			else
				throw std::runtime_error("Failed to find a suitable GPU");
		}

		int indice = GetAvailableQueueFamily(g_physicalDevice, desiredQueueCapabilities);
		float queuePriority = 1.0f;
		vk::DeviceQueueCreateInfo queueCreateInfo(
			vk::DeviceQueueCreateFlags(),
			indice,
			1,
			&queuePriority
		);

		vk::PhysicalDeviceFeatures deviceFeatures = {};

		vk::DeviceCreateInfo createInfo(
			vk::DeviceCreateFlags(),
			1,
			&queueCreateInfo,
			static_cast<uint32_t>(g_validationLayers.size()),
			g_validationLayers.data(),
			0,
			nullptr,
			&deviceFeatures
		);

		CHECK_VK_RESULT_FATAL(g_physicalDevice.createDevice(&createInfo, g_allocator, &g_device), "Failed to create logical device");

		g_graphicsQueue.index = 0;
		g_graphicsQueue.queue = g_device.getQueue(indice, g_graphicsQueue.index);
	}

	static void InitVulkan()
	{
		CreateInstance();
		InitDevice();
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

			g_device.destroy(g_allocator);

			#ifndef NDEBUG
			g_instance.destroyDebugUtilsMessengerEXT(g_debugMessenger, g_allocator, g_dldy);
			#endif

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