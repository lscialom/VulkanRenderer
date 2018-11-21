#pragma once

#include <vulkan/vulkan.hpp>

extern vk::Instance g_instance;
extern vk::PhysicalDevice g_physicalDevice;

// TODO Make a Device class and move it along allocator ?
extern vk::Device g_device;

extern vk::SurfaceKHR g_surface;

extern vk::Format g_requiredFormat;
extern vk::Extent2D g_extent;

struct Queue {
  int index = -1;
  vk::Queue handle = nullptr;
};

extern Queue g_graphicsQueue;
extern Queue g_presentQueue;

extern vk::CommandPool g_commandPool;

// extern vk::DescriptorSetLayout g_baseDescriptorSetLayout;

extern vk::RenderPass g_renderPass;

extern vk::AllocationCallbacks *g_allocationCallbacks;

extern vk::DispatchLoaderDynamic g_dldy;

#ifndef NDEBUG
extern std::vector<vk::DebugUtilsMessengerEXT> g_debugMessengers;
#endif
