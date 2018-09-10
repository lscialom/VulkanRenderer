#include "renderer.hpp"
#include "window_handler.hpp"

#include <vulkan/vulkan.hpp>

#include <iostream>

#include <vector>
#include <map>
#include <set>

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
	static uint32_t g_width, g_height = 0;

	static vk::Instance g_instance;
	static vk::PhysicalDevice g_physicalDevice;

	static const std::vector<const char*> g_deviceExtensions = {
		VK_KHR_SWAPCHAIN_EXTENSION_NAME
	};

	static vk::Device g_device;

	static vk::SurfaceKHR g_surface;

	static struct
	{
		vk::SwapchainKHR handle;

		std::vector<vk::Image> images;
		std::vector<vk::ImageView> imageViews;

		vk::Format format;
		vk::Extent2D extent;

	} g_swapchain;

	struct SwapChainSupportDetails
	{
		vk::SurfaceCapabilitiesKHR capabilities;
		std::vector<vk::SurfaceFormatKHR> formats;
		std::vector<vk::PresentModeKHR> presentModes;
	};

	struct QueueFamilyIndices
	{
		int graphicsFamily = -1;
		int presentFamily = -1;

		bool isComplete()
		{
			return graphicsFamily >= 0 && presentFamily >= 0;
		}
	};

	struct Queue
	{
		int index = -1;
		vk::Queue handle = nullptr;
	};

	static Queue g_graphicsQueue;
	static Queue g_presentQueue;

	static vk::AllocationCallbacks* g_allocator = nullptr;

	static vk::DispatchLoaderDynamic g_dldy;

	#ifndef NDEBUG
	static const std::vector<const char*> g_validationLayers = {
		"VK_LAYER_LUNARG_standard_validation",
		"VK_LAYER_LUNARG_monitor"
	};

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

	static void SetMainObjectsDebugNames()
	{
		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::eInstance, (uint64_t)((VkInstance)g_instance), "Vulkan Instance" }, g_dldy);

		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::eSurfaceKHR, (uint64_t)((VkSurfaceKHR)g_surface), "Window Surface" }, g_dldy);

		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::ePhysicalDevice, (uint64_t)((VkPhysicalDevice)g_physicalDevice), (std::string("Physical Device - ") + g_physicalDevice.getProperties().deviceName).c_str() }, g_dldy);
		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::eDevice, (uint64_t)((VkDevice)g_device), (std::string("Logical Device - ") + g_physicalDevice.getProperties().deviceName).c_str() }, g_dldy);

		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::eQueue, (uint64_t)((VkQueue)g_graphicsQueue.handle), "Graphics Queue" }, g_dldy);
		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::eQueue, (uint64_t)((VkQueue)g_presentQueue.handle), "Present Queue" }, g_dldy);

		g_device.setDebugUtilsObjectNameEXT({ vk::ObjectType::eDebugUtilsMessengerEXT, (uint64_t)((VkDebugUtilsMessengerEXT)g_debugMessenger), "Debug Messenger" }, g_dldy);
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
			vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose | vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning | vk::DebugUtilsMessageSeverityFlagBitsEXT::eError, //Add eInfo for info logs
			vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral | vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation | vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance,
			g_debugCallback
		);

		g_debugMessenger = g_instance.createDebugUtilsMessengerEXT(dbgCreateInfo, g_allocator, g_dldy);
		#endif
	}

	static QueueFamilyIndices GetQueueFamilies(vk::PhysicalDevice device)
	{
		std::vector<vk::QueueFamilyProperties> queueFamilies = device.getQueueFamilyProperties();
		QueueFamilyIndices indices;

		int i = 0;
		for (const auto& queueFamily : queueFamilies)
		{
			if (queueFamily.queueCount > 0)
			{
				if (queueFamily.queueFlags & vk::QueueFlagBits::eGraphics)
					indices.graphicsFamily = i;

				if (device.getSurfaceSupportKHR(i, g_surface))
					indices.presentFamily = i;
			}

			if (indices.isComplete())
				break;

			i++;
		}

		return indices;
	}

	static bool CheckDeviceExtensionSupport(vk::PhysicalDevice device)
	{
		const std::vector<vk::ExtensionProperties> availableExtensions = device.enumerateDeviceExtensionProperties();

		std::set<std::string> requiredExtensions(g_deviceExtensions.begin(), g_deviceExtensions.end());

		for (const auto& extension : availableExtensions)
			requiredExtensions.erase(extension.extensionName);

		return requiredExtensions.empty();
	}

	static SwapChainSupportDetails QuerySwapChainSupport(vk::PhysicalDevice device)
	{
		SwapChainSupportDetails details;

		details.capabilities = device.getSurfaceCapabilitiesKHR(g_surface);
		details.formats = device.getSurfaceFormatsKHR(g_surface);
		details.presentModes = device.getSurfacePresentModesKHR(g_surface);

		return details;
	}

	static int RateDeviceSuitability(vk::PhysicalDevice device)
	{
		if (!GetQueueFamilies(device).isComplete() || !CheckDeviceExtensionSupport(device))
			return 0;

		//Done separately since we need to check for the swapchain extension support first
		SwapChainSupportDetails swapChainSupport = QuerySwapChainSupport(device);
		if (swapChainSupport.formats.empty() || swapChainSupport.presentModes.empty())
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
		const std::vector<vk::PhysicalDevice> devices = g_instance.enumeratePhysicalDevices();

		if (devices.empty())
			throw std::runtime_error("failed to find GPUs with Vulkan support!");

		{
			// Use an ordered map to automatically sort candidates by increasing score
			std::multimap<int, vk::PhysicalDevice> candidates;

			for (const auto& device : devices)
			{
				int score = RateDeviceSuitability(device);
				candidates.insert(std::make_pair(score, device));
			}

			// Check if the best candidate is suitable at all
			if (candidates.rbegin()->first > 0)
				g_physicalDevice = candidates.rbegin()->second;
			else
				throw std::runtime_error("Failed to find a suitable GPU");
		}

		std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
		const QueueFamilyIndices indices = GetQueueFamilies(g_physicalDevice);
		const std::set<int> uniqueQueueFamilies = { indices.graphicsFamily, indices.presentFamily };

		float queuePriority = 1.0f;
		for (int queueFamily : uniqueQueueFamilies)
		{
			vk::DeviceQueueCreateInfo queueCreateInfo(
				vk::DeviceQueueCreateFlags(),
				queueFamily,
				1,
				&queuePriority
			);

			queueCreateInfos.push_back(queueCreateInfo);
		}

		vk::PhysicalDeviceFeatures deviceFeatures = {};

		vk::DeviceCreateInfo createInfo(
			vk::DeviceCreateFlags(),
			static_cast<uint32_t>(queueCreateInfos.size()),
			queueCreateInfos.data(),
			#ifndef NDEBUG
			static_cast<uint32_t>(g_validationLayers.size()),
			g_validationLayers.data(),
			#else
			0,
			nullptr,
			#endif
			static_cast<uint32_t>(g_deviceExtensions.size()),
			g_deviceExtensions.data(),
			&deviceFeatures
		);

		CHECK_VK_RESULT_FATAL(g_physicalDevice.createDevice(&createInfo, g_allocator, &g_device), "Failed to create logical device");

		g_dldy.init(g_instance, g_device);

		g_graphicsQueue.index = 0;
		g_graphicsQueue.handle = g_device.getQueue(indices.graphicsFamily, g_graphicsQueue.index);

		g_presentQueue.index = 0;
		g_presentQueue.handle = g_device.getQueue(indices.presentFamily, g_presentQueue.index);
	}

	static void InitSwapchain()
	{
		SwapChainSupportDetails swapChainSupport = QuerySwapChainSupport(g_physicalDevice);

		vk::SurfaceFormatKHR format = swapChainSupport.formats[0]; //safe since it has been checked that there are available formats
		if (swapChainSupport.formats.size() == 1 && swapChainSupport.formats[0].format == vk::Format::eUndefined)
			format = { vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear };
		else
		{
			for (const auto& availableFormat : swapChainSupport.formats)
			{
				if (availableFormat.format == vk::Format::eB8G8R8A8Unorm && availableFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear)
					format = availableFormat;
			}
		}

		vk::PresentModeKHR presentMode = vk::PresentModeKHR::eFifo;
		for (const auto& availablePresentMode : swapChainSupport.presentModes)
		{
			if (availablePresentMode == vk::PresentModeKHR::eMailbox)
			{
				presentMode = availablePresentMode;
				break;
			}
			else if (availablePresentMode == vk::PresentModeKHR::eImmediate)
				presentMode = availablePresentMode;
		}

		vk::Extent2D extent;
		if (swapChainSupport.capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
		{
			extent = swapChainSupport.capabilities.currentExtent;
		}
		else
		{
			VkExtent2D actualExtent = { g_width, g_height };

			actualExtent.width = std::max(swapChainSupport.capabilities.minImageExtent.width, std::min(swapChainSupport.capabilities.maxImageExtent.width, actualExtent.width));
			actualExtent.height = std::max(swapChainSupport.capabilities.minImageExtent.height, std::min(swapChainSupport.capabilities.maxImageExtent.height, actualExtent.height));

			extent = actualExtent;
		}

		uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
		if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount)
			imageCount = swapChainSupport.capabilities.maxImageCount;

		QueueFamilyIndices indices = GetQueueFamilies(g_physicalDevice);
		uint32_t queueFamilyIndices[] = { (uint32_t)indices.graphicsFamily, (uint32_t)indices.presentFamily };

		vk::SharingMode imageSharingMode = vk::SharingMode::eExclusive;
		uint32_t queueFamilyIndexCount = 0;

		if (indices.graphicsFamily != indices.presentFamily)
		{
			imageSharingMode = vk::SharingMode::eConcurrent;
			queueFamilyIndexCount = sizeof(queueFamilyIndices) / sizeof(uint32_t);
		}

		vk::SwapchainCreateInfoKHR createInfo(
			vk::SwapchainCreateFlagsKHR(),
			g_surface,
			imageCount,
			format.format,
			format.colorSpace,
			extent,
			1,
			vk::ImageUsageFlagBits::eColorAttachment,
			imageSharingMode,
			queueFamilyIndexCount,
			queueFamilyIndices,
			swapChainSupport.capabilities.currentTransform,
			vk::CompositeAlphaFlagBitsKHR::eOpaque,
			presentMode,
			VK_TRUE
			//Old swapchain
		);

		g_device.createSwapchainKHR(&createInfo, g_allocator, &g_swapchain.handle);

		g_swapchain.images = g_device.getSwapchainImagesKHR(g_swapchain.handle);
		g_swapchain.format = format.format;
		g_swapchain.extent = extent;

		vk::ImageSubresourceRange imageSubresourceRange(
			vk::ImageAspectFlagBits::eColor,
			0,
			1,
			0,
			1
		);

		g_swapchain.imageViews.resize(g_swapchain.images.size());
		for (size_t i = 0; i < g_swapchain.images.size(); i++)
		{
			vk::ImageViewCreateInfo createInfo(
				vk::ImageViewCreateFlags(),
				g_swapchain.images[i],
				vk::ImageViewType::e2D,
				g_swapchain.format,
				vk::ComponentMapping(),
				imageSubresourceRange
			);

			CHECK_VK_RESULT_FATAL(g_device.createImageView(&createInfo, g_allocator, &g_swapchain.imageViews[i]), "Failed to create image views");
		}
	}

	static void InitVulkan()
	{
		CreateInstance();
		CHECK_VK_RESULT_FATAL((vk::Result)WindowHandler::CreateSurface((VkInstance)g_instance, (VkAllocationCallbacks*)g_allocator, (VkSurfaceKHR*)&g_surface), "Failed to create window surface");
		InitDevice();
		InitSwapchain(); //TODO Set debug names

		#ifndef NDEBUG
		SetMainObjectsDebugNames();
		#endif
	}

	void Init(unsigned int width, unsigned int height)
	{
		assert(width != 0 && height != 0);

		g_width = width, g_height = height;

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
			for (auto imageView : g_swapchain.imageViews)
				g_device.destroyImageView(imageView, g_allocator);

			g_device.destroySwapchainKHR(g_swapchain.handle, g_allocator);

			g_device.destroy(g_allocator);

			#ifndef NDEBUG
			g_instance.destroyDebugUtilsMessengerEXT(g_debugMessenger, g_allocator, g_dldy);
			#endif

			g_instance.destroySurfaceKHR(g_surface, g_allocator);
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