#include "global_context.hpp"

vk::Instance g_instance = nullptr;
vk::PhysicalDevice g_physicalDevice = nullptr;

// TODO Make a Device class and move it along allocator ?
vk::Device g_device = nullptr;

vk::SurfaceKHR g_surface = nullptr;

vk::CommandPool g_commandPool = nullptr;

// vk::DescriptorSetLayout g_baseDescriptorSetLayout;

vk::RenderPass g_renderPass = nullptr;

vk::AllocationCallbacks *g_allocationCallbacks = nullptr;

vk::DispatchLoaderDynamic g_dldy = {};

#ifndef NDEBUG
std::vector<vk::DebugUtilsMessengerEXT> g_debugMessengers = {};
#endif