#pragma once

#include <vulkan/vulkan.hpp>

//TODO Move this somewhere else
struct Queue {
  int index = -1;
  vk::Queue handle = nullptr;
};

extern vk::Instance g_instance;
extern vk::PhysicalDevice g_physicalDevice;

// TODO Make a Device class and move it along allocator ?
extern vk::Device g_device;

extern vk::SurfaceKHR g_surface;

extern vk::CommandPool g_commandPool;

// extern vk::DescriptorSetLayout g_baseDescriptorSetLayout;

// TODO Should be elsewhere than global
extern vk::RenderPass g_renderPass;

extern vk::AllocationCallbacks *g_allocationCallbacks;

extern vk::DispatchLoaderDynamic g_dldy;

#ifndef NDEBUG
extern std::vector<vk::DebugUtilsMessengerEXT> g_debugMessengers;
#endif