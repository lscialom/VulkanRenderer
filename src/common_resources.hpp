#pragma once

#include "data_structures.hpp"
#include "descriptors.hpp"
#include "maths.hpp"
#include "memory.hpp"

#include <vulkan/vulkan.hpp>

#include <array>

namespace Renderer {
namespace CommonResources {

//#include "descriptor_layouts.inl"
extern ::Renderer::DescriptorLayout UniqueTextureLayout;
extern ::Renderer::DescriptorLayout MeshLayout;
extern ::Renderer::DescriptorLayout GBufferLayout;
extern ::Renderer::DescriptorLayout SSAOLayout;

void InitDescriptorLayouts();
void DestroyDescriptorLayouts();

//#include "uniform_buffer_objects.inl"
extern UniformBufferObject CameraUBOSet;
extern UniformBufferObject LightUBOSet;

void InitUniformBufferObjects();
void DestroyUniformBufferObjects();

//#include "samplers.inl"
extern vk::Sampler BaseSampler;
extern vk::Sampler TextureSampler;
extern vk::Sampler RepeatSampler;

void InitSamplers();
void DestroySamplers();

//#include "postprocess_resources.inl"
extern ::Renderer::Buffer SSAOKernelBuffer;
extern ::Renderer::Image SSAONoiseTex;
extern ::Renderer::Image DitherTex;

void InitPostProcessResources();
void DestroyPostProcessResources();

} // namespace CommonResources
} // namespace Renderer
