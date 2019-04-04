#include "renderer.hpp"

#include "global_context.hpp"

#include "swapchain.hpp"

#include "configuration_helper.hpp"
#include "data_structures.hpp"
#include "maths.hpp"
#include "memory.hpp"

#include "common_resources.hpp"
#include "pass.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

//-----------------------------------------------------------------------------
// USER CONFIG VARIABLES
//-----------------------------------------------------------------------------
namespace Renderer {
namespace Config {

bool ShowGBuffer = true;
bool SSAOEnable = true;

} // namespace Config
} // namespace Renderer

//-----------------------------------------------------------------------------
// CAMERA
//-----------------------------------------------------------------------------
namespace Renderer {
namespace Camera {

float Fov = 90.f;
float Near = 0.1f;
float Far = 100.f;

Vec3 Position = Vec3::Zero();
Vec3 Rotation = Vec3::Zero();

Eigen::Matrix4f Matrix;
Eigen::Matrix4f ProjMatrix;
Eigen::Matrix4f ViewMatrix;

static void UpdateMatrix() {
  Eigen::AngleAxisf rotX, rotY, rotZ;

  Eigen::Vector3f position = Maths::EigenizeVec3(Position);

  rotX = Eigen::AngleAxisf(Rotation.x, Eigen::Vector3f::UnitX());
  rotY = Eigen::AngleAxisf(Rotation.y, Eigen::Vector3f::UnitY());
  rotZ = Eigen::AngleAxisf(Rotation.z, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf rot = rotZ * rotY * rotX;

  vk::Extent2D extent = Swapchain::GetExtent();

  ViewMatrix =
      Maths::LookAt(position, position + rot * Eigen::Vector3f::UnitZ(),
                    Eigen::Vector3f::UnitY());
  ProjMatrix =
      Maths::Perspective(Camera::Fov, extent.width / (float)extent.height,
                         Camera::Near, Camera::Far);

  ProjMatrix(1, 1) *= -1;

  Matrix = ProjMatrix * ViewMatrix;
}

} // namespace Camera
} // namespace Renderer

namespace Renderer {
//-----------------------------------------------------------------------------
// RENDER CONTEXT
//-----------------------------------------------------------------------------

static uint32_t requestedWidth = 0;
static uint32_t requestedHeight = 0;

Queue graphicsQueue;

std::array<vk::Semaphore, MAX_IN_FLIGHT_FRAMES> renderFinishedSemaphores;

struct Texture {
private:
  Image image;

public:
  void init(const std::string &path) {
    int texWidth, texHeight, texChannels;
    stbi_uc *pixels =
        stbi_load(path.c_str(), &texWidth, &texHeight, &texChannels,
                  STBI_rgb_alpha); // TODO Support multiple pixel formats

    if (!pixels) {
      printf("[Error] Failed to load texture image %s\n", path.c_str());
      return;
    }

    image.allocate(texWidth, texHeight, vk::Format::eR8G8B8A8Unorm,
                   vk::ImageTiling::eOptimal,
                   vk::ImageUsageFlagBits::eTransferDst |
                       vk::ImageUsageFlagBits::eSampled,
                   vk::MemoryPropertyFlagBits::eDeviceLocal);

    // cast for correct template deduction (for inner staging buffer size
    // deduction)
    // TODO Support multiple image formats
    image.write_from_raw_data(reinterpret_cast<uint32_t *>(pixels),
                              vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eShaderReadOnlyOptimal);
  }
};

struct RenderContext {
  std::vector<vk::CommandBuffer> commandbuffers;

  std::vector<Model *> models;
  std::vector<Light *> lights;

  Pass gPass;
  Pass ssaoPass;
  Pass blurPass;
  Pass lightPass;

  vk::PhysicalDeviceRayTracingPropertiesNV rtProperties = {};

  void init_ray_tracing() {
    vk::PhysicalDeviceProperties2 pdProperties;
    pdProperties.pNext = &rtProperties;
    pdProperties.properties = {};

    g_physicalDevice.getProperties2(&pdProperties);
  }

  void init_g_pass() {
    vk::Extent2D extent = Swapchain::GetExtent();

    std::vector<AttachmentInfo> attachmentInfos;
    attachmentInfos.resize(G_BUFFER_SIZE);

    attachmentInfos[DEPTH_BUFFER_INDEX].extent = extent;
    attachmentInfos[DEPTH_BUFFER_INDEX].format = FindSupportedFormat(
        g_physicalDevice,
        {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint,
         vk::Format::eD24UnormS8Uint},
        vk::ImageTiling::eOptimal,
        vk::FormatFeatureFlagBits::eDepthStencilAttachment);

    attachmentInfos[DEPTH_BUFFER_INDEX].isDepth = true;

    for (size_t i = 1; i < attachmentInfos.size(); ++i) {
      attachmentInfos[i].extent = extent;
      attachmentInfos[i].format = vk::Format::eR32G32B32A32Sfloat;
      attachmentInfos[i].isDepth = false;
    }

    std::vector<vk::PushConstantRange> pushConstantRanges;

    PushConstantDescriptor<ModelInstancePushConstant>
        modelInstancePushConstantRange{};
    pushConstantRanges.push_back(
        modelInstancePushConstantRange.make_push_constant_range());

    std::vector<UniformBufferObject *> ubos;
    ubos.push_back(&CommonResources::CameraUBOSet);

    ShaderInfo shaderInfo;
    shaderInfo.vertPath = "../resources/shaders/spv/geom.vert.spv";
    shaderInfo.fragPath = "../resources/shaders/spv/geom.frag.spv";
    shaderInfo.useVertexInput = true;
    shaderInfo.cull = true;
    shaderInfo.blendEnable = false;
    shaderInfo.drawModels = true;
    shaderInfo.pushConstants = pushConstantRanges;
    shaderInfo.ubos = ubos;

    gPass.init_offscreen(extent, attachmentInfos, {shaderInfo});
  }

  void init_ssao_pass() {
    vk::Extent2D extent = Swapchain::GetExtent();

    AttachmentInfo attachmentInfo;
    attachmentInfo.extent = extent;
    attachmentInfo.format = vk::Format::eR32Sfloat;
    attachmentInfo.isDepth = false;

    std::vector<const Image *> images;
    images.resize(G_BUFFER_SIZE);

    images[POS_BUFFER_INDEX] = &gPass.get_attachment_image(POS_BUFFER_INDEX);
    images[NORMAL_BUFFER_INDEX] =
        &gPass.get_attachment_image(NORMAL_BUFFER_INDEX);
    images[COLOR_BUFFER_INDEX] =
        &gPass.get_attachment_image(COLOR_BUFFER_INDEX);
    images[DEPTH_BUFFER_INDEX] =
        &gPass.get_attachment_image(DEPTH_BUFFER_INDEX);

    std::vector<const vk::Sampler *> samplers(G_BUFFER_SIZE,
                                              &CommonResources::BaseSampler);

    std::vector<UniformBufferObject *> ubos;
    ubos.push_back(&CommonResources::CameraUBOSet);

    std::vector<vk::PushConstantRange> pushConstantRanges;

    PushConstantDescriptor<SSAOPassPushConstant> ssaoPassPushConstantRange{};
    pushConstantRanges.push_back(
        ssaoPassPushConstantRange.make_push_constant_range());

    std::vector<DescriptorSetInfo> descriptorSetInfos;
    descriptorSetInfos.resize(2);

    descriptorSetInfos[0].layout = CommonResources::GBufferLayout;
    descriptorSetInfos[0].images = images;
    descriptorSetInfos[0].samplers = samplers;

    descriptorSetInfos[1].layout = CommonResources::SSAOLayout;
    descriptorSetInfos[1].buffers = {&CommonResources::SSAOKernelBuffer};
    descriptorSetInfos[1].images = {&CommonResources::SSAONoiseTex};
    descriptorSetInfos[1].samplers = {&CommonResources::RepeatSampler};

    ShaderInfo shaderInfo;
    shaderInfo.vertPath = "../resources/shaders/spv/fullscreen.vert.spv";
    shaderInfo.fragPath = "../resources/shaders/spv/ssao.frag.spv";
    shaderInfo.useVertexInput = false;
    shaderInfo.cull = false;
    shaderInfo.blendEnable = false;
    shaderInfo.drawModels = false;
    shaderInfo.pushConstants = pushConstantRanges;
    shaderInfo.descriptors = descriptorSetInfos;
    shaderInfo.ubos = ubos;

    ssaoPass.init_offscreen(extent, {attachmentInfo}, {shaderInfo});
  }

  void init_blur_pass() {
    vk::Extent2D extent = Swapchain::GetExtent();

    AttachmentInfo attachmentInfo;
    attachmentInfo.extent = extent;
    attachmentInfo.format = vk::Format::eR32Sfloat;
    attachmentInfo.isDepth = false;

    DescriptorSetInfo descriptorSetInfo;
    descriptorSetInfo.layout = CommonResources::UniqueTextureLayout;
    descriptorSetInfo.samplers = {&CommonResources::BaseSampler};
    descriptorSetInfo.images = {&ssaoPass.get_attachment_image()};

    ShaderInfo shaderInfo;
    shaderInfo.vertPath = "../resources/shaders/spv/fullscreen.vert.spv";
    shaderInfo.fragPath = "../resources/shaders/spv/blur.frag.spv";
    shaderInfo.useVertexInput = false;
    shaderInfo.cull = false;
    shaderInfo.blendEnable = false;
    shaderInfo.drawModels = false;
    shaderInfo.descriptors = {descriptorSetInfo};

    blurPass.init_offscreen(extent, {attachmentInfo}, {shaderInfo});
  }

  void init_light_pass() {
    std::vector<const Image *> images;
    images.resize(G_BUFFER_SIZE);

    images[POS_BUFFER_INDEX] = &gPass.get_attachment_image(POS_BUFFER_INDEX);
    images[NORMAL_BUFFER_INDEX] =
        &gPass.get_attachment_image(NORMAL_BUFFER_INDEX);
    images[COLOR_BUFFER_INDEX] =
        &gPass.get_attachment_image(COLOR_BUFFER_INDEX);
    images[DEPTH_BUFFER_INDEX] =
        &gPass.get_attachment_image(DEPTH_BUFFER_INDEX);

    std::vector<const vk::Sampler *> samplers(G_BUFFER_SIZE,
                                              &CommonResources::BaseSampler);

    vk::PushConstantRange lightShaderPushConstant;
    lightShaderPushConstant = PushConstantDescriptor<LightPassPushConstant>()
                                  .make_push_constant_range();

    std::vector<UniformBufferObject *> ubos;

    ubos.push_back(&CommonResources::CameraUBOSet);
    ubos.push_back(&CommonResources::LightUBOSet);

    std::vector<DescriptorSetInfo> descriptorSetInfos;
    descriptorSetInfos.resize(3);

    descriptorSetInfos[0].layout = CommonResources::GBufferLayout;
    descriptorSetInfos[0].images = images;
    descriptorSetInfos[0].samplers = samplers;

    descriptorSetInfos[1].layout = CommonResources::UniqueTextureLayout;
    descriptorSetInfos[1].images = {&blurPass.get_attachment_image()};
    descriptorSetInfos[1].samplers = {&CommonResources::BaseSampler};

    descriptorSetInfos[2].layout = CommonResources::UniqueTextureLayout;
    descriptorSetInfos[2].images = {&CommonResources::DitherTex};
    descriptorSetInfos[2].samplers = {&CommonResources::RepeatSampler};

    std::vector<ShaderInfo> shaderInfos;
    shaderInfos.resize(2);

    shaderInfos[0].vertPath = "../resources/shaders/spv/fullscreen.vert.spv";
    shaderInfos[0].fragPath = "../resources/shaders/spv/light.frag.spv";
    shaderInfos[0].useVertexInput = false;
    shaderInfos[0].cull = false;
    shaderInfos[0].blendEnable = true;
    shaderInfos[0].drawModels = false;
    shaderInfos[0].pushConstants = {lightShaderPushConstant};
    shaderInfos[0].descriptors = descriptorSetInfos;
    shaderInfos[0].ubos = ubos;

    vk::PushConstantRange overlayShaderPushConstant;
    overlayShaderPushConstant =
        PushConstantDescriptor<ExtentPushConstant>().make_push_constant_range();

    shaderInfos[1].vertPath = "../resources/shaders/spv/overlay.vert.spv";
    shaderInfos[1].fragPath = "../resources/shaders/spv/overlay.frag.spv";
    shaderInfos[1].useVertexInput = false;
    shaderInfos[1].cull = false;
    shaderInfos[1].blendEnable = true;
    shaderInfos[1].drawModels = false;
    shaderInfos[1].pushConstants = {overlayShaderPushConstant};
    shaderInfos[1].descriptors = {descriptorSetInfos[0]};
    shaderInfos[1].drawRectCount = G_BUFFER_SIZE;

    lightPass.init(shaderInfos);
  }

  void destroy_g_pass() { gPass.destroy(); }

  void destroy_ssao_pass() { ssaoPass.destroy(); }

  void destroy_blur_pass() { blurPass.destroy(); }

  void destroy_light_pass() { lightPass.destroy(); }

  void init_commandbuffers() {
    commandbuffers.resize(Swapchain::ImageCount());

    vk::CommandBufferAllocateInfo allocInfo(g_commandPool,
                                            vk::CommandBufferLevel::ePrimary,
                                            (uint32_t)commandbuffers.size());

    CHECK_VK_RESULT_FATAL(
        g_device.allocateCommandBuffers(&allocInfo, commandbuffers.data()),
        "Failed to allocate command buffers.");
  }

  void destroy_commandbuffers() {
    g_device.freeCommandBuffers(g_commandPool,
                                static_cast<uint32_t>(commandbuffers.size()),
                                commandbuffers.data());
  }

  void update_commandbuffer(size_t index) {
    vk::CommandBufferBeginInfo beginInfo(
        vk::CommandBufferUsageFlagBits::eSimultaneousUse, nullptr);

    vk::Extent2D extent = Swapchain::GetExtent();
    vk::Viewport viewport(0, 0, (float)extent.width, (float)extent.height, 0,
                          1);
    vk::Rect2D scissor({}, extent);

    static constexpr const std::array<float, 4> blackColorArray = {
        {0.0f, 0.0f, 0.0f, 1.0f}};

    vk::ClearColorValue blackColorValue = blackColorArray;
    vk::ClearDepthStencilValue clearDepthValue = {1.0f};

    std::vector<vk::ClearValue> clearColors = {
        clearDepthValue, blackColorValue, blackColorValue, blackColorValue};

    CHECK_VK_RESULT_FATAL(commandbuffers[index].begin(&beginInfo),
                          "Failed to begin recording command buffer.");

    commandbuffers[index].setViewport(0, 1, &viewport);
    commandbuffers[index].setScissor(0, 1, &scissor);

    CameraUBO cam{};
    cam.view = Camera::ViewMatrix;
    cam.proj = Camera::ProjMatrix;
    cam.viewPos = Maths::EigenizeVec3(Camera::Position);

    CommonResources::CameraUBOSet.write(index, &cam, sizeof(cam), 0);

    gPass.record(commandbuffers[index], index, {}, clearColors, models);

    static constexpr const std::array<float, 4> whiteClearColorArray = {
        {1.0f, 1.0f, 1.0f, 1.0f}};

    vk::ClearColorValue whiteClearColorValue = whiteClearColorArray;
    vk::ClearValue whiteClearValue = (vk::ClearValue)whiteClearColorValue;

    // ssao pass
    if (Config::SSAOEnable) {

      SSAOPassPushConstant ssaoPassPushConstant{};
      ssaoPassPushConstant.xExtent = extent.width;
      ssaoPassPushConstant.yExtent = extent.height;

      ssaoPass.record(commandbuffers[index], index, {{&ssaoPassPushConstant}},
                      {whiteClearValue});
      blurPass.record(commandbuffers[index], index, {}, {whiteClearValue});

    } else
      blurPass.record_clear_attachments(commandbuffers[index], whiteClearValue);

    // TODO WRAP INTO LIGHT STRUCT
    // std::vector<LightUBO> lightsUBOData;
    for (size_t i = 0; i < lights.size(); ++i) {
      LightUBO ubo{};

      ubo.vector.head<3>() << Maths::EigenizeVec3(lights[i]->vector);

      // Apply translation only if vector represents a position
      ubo.vector.tail<1>() << (float)lights[i]->lightType;

      ubo.vector = Camera::ViewMatrix * ubo.vector;

      ubo.color = Maths::EigenizeVec3(lights[i]->color);
      ubo.ambientFactor = lights[i]->ambientFactor;
      ubo.maxDist = lights[i]->maxDist;

      CommonResources::LightUBOSet.write(index, &ubo, sizeof(LightUBO), i);

      // lightsUBOData.push_back(ubo);
    }

    // TODO Support alignment for multiple elements at once
    // lightsUBO.write(index, lightsUBOData.data(),
    //                sizeof(LightUBO) * lightsUBOData.size(), 0);

    // light pass
    vk::ClearValue clearValue = (vk::ClearValue)blackColorValue;

    std::vector<std::vector<void *>> lightPassPushConstants;
    lightPassPushConstants.resize(2);

    LightPassPushConstant lightPassPushConstant{};
    lightPassPushConstant.numLights = lights.size();

    ExtentPushConstant ePc = {};
    ePc.xExtent = extent.width;
    ePc.yExtent = extent.height;
    ePc.ratio = 1.f / 6.f;

    lightPassPushConstants[0].push_back(&lightPassPushConstant);
    lightPassPushConstants[1].push_back(&ePc);

    lightPass.set_shader_enabled("overlay", Config::ShowGBuffer);

    lightPass.record(commandbuffers[index], index, lightPassPushConstants,
                     {clearValue});

    commandbuffers[index]
        .end(); // Strangely, it does returns void instead of vk::Result so no
                // error checking is possible here
  }

  void init() {
    Swapchain::Init(requestedWidth, requestedHeight);

    CommonResources::InitDescriptorLayouts();
    CommonResources::InitUniformBufferObjects();

    CommonResources::InitSamplers();
    CommonResources::InitPostProcessResources();

    init_ray_tracing();

    init_g_pass();
    init_ssao_pass();
    init_blur_pass();
    init_light_pass();

    init_commandbuffers();
  }

  void destroy(bool freeModels = true) {

    CommonResources::DestroyDescriptorLayouts();
    CommonResources::DestroyUniformBufferObjects();

    CommonResources::DestroySamplers();
    CommonResources::DestroyPostProcessResources();

    destroy_commandbuffers();

    destroy_light_pass();
    destroy_blur_pass();
    destroy_ssao_pass();
    destroy_g_pass();

    Swapchain::Destroy();

    if (!freeModels)
      return;

    for (size_t j = 0; j < models.size(); ++j)
      delete models[j];

    models.clear();

    for (size_t j = 0; j < lights.size(); ++j)
      delete lights[j];

    lights.clear();
  }

  void refresh() {
    g_device.waitIdle();

    destroy_commandbuffers();

    destroy_light_pass();
    destroy_blur_pass();
    destroy_ssao_pass();
    destroy_g_pass();

    Swapchain::Destroy();
    Swapchain::Init(requestedWidth, requestedHeight);

    init_commandbuffers();

    init_g_pass();
    init_ssao_pass();
    init_blur_pass();
    init_light_pass();

    // TODO UPDATE SHADERS INSTEAD OF RECREATING THEM
  }

  Light *spawn_light(Vec3 position, Vec3 color, float ambientStrength) {

    Light *light = new Light(position, color, ambientStrength);
    lights.push_back(light);

    return lights.back();
  }

  // void update_transforms(uint32_t imageIndex) {
  //  for (const auto &pair : models) {
  //    for (size_t j = 0; j < pair.second.size(); ++j) {
  //      pair.second[j]->update_mvp(imageIndex);
  //    }
  //  }
  //}
};

static RenderContext renderContext;

//-----------------------------------------------------------------------------
// DEBUG UTILS
//-----------------------------------------------------------------------------

// TODO Replace with sth more convenient
#ifndef NDEBUG
static void SetMainObjectsDebugNames() {
  g_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eInstance,
                                       (uint64_t)((VkInstance)g_instance),
                                       "Vulkan Instance"},
                                      g_dldy);

  g_device.setDebugUtilsObjectNameEXT({vk::ObjectType::eSurfaceKHR,
                                       (uint64_t)((VkSurfaceKHR)g_surface),
                                       "Window Surface"},
                                      g_dldy);

  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::ePhysicalDevice,
       (uint64_t)((VkPhysicalDevice)g_physicalDevice),
       (std::string("Physical Device - ") +
        g_physicalDevice.getProperties().deviceName)
           .c_str()},
      g_dldy);
  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eDevice, (uint64_t)((VkDevice)g_device),
       (std::string("Logical Device - ") +
        g_physicalDevice.getProperties().deviceName)
           .c_str()},
      g_dldy);

  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eQueue, (uint64_t)((VkQueue)graphicsQueue.handle),
       "Graphics Queue"},
      g_dldy);

  // Swapchain done in swapchain.cpp

  for (size_t i = 0; i < g_debugMessengers.size(); ++i)
    g_device.setDebugUtilsObjectNameEXT(
        {vk::ObjectType::eDebugUtilsMessengerEXT,
         (uint64_t)((VkDebugUtilsMessengerEXT)g_debugMessengers[i]),
         g_debugMessengersInfos[i].name},
        g_dldy);
}
#endif

//-----------------------------------------------------------------------------
// DRAW
//-----------------------------------------------------------------------------

static void Draw() {
  vk::Result result = Swapchain::AcquireNextImage();

  if (result == vk::Result::eErrorOutOfDateKHR) {
    renderContext.refresh();
    return;
  } else if (result != vk::Result::eSuccess &&
             result != vk::Result::eSuboptimalKHR) {
    std::string err = "[FATAL]";
    err += "Failed to acquire Swapchain image.";
    err += " : ";
    err += vk::to_string(result);

    throw std::runtime_error(err.c_str());
  }

  vk::PipelineStageFlags waitStage =
      vk::PipelineStageFlagBits::eColorAttachmentOutput;

  uint32_t imageIndex = Swapchain::GetCurrentImageIndex();
  size_t currentFrame = Swapchain::GetCurrentFrameIndex();

  Camera::UpdateMatrix();

  renderContext.update_commandbuffer(imageIndex);

  vk::SubmitInfo submitInfo(1, Swapchain::GetCurrentImageSemaphore(),
                            &waitStage, 1,
                            &renderContext.commandbuffers[imageIndex], 1,
                            &renderFinishedSemaphores[currentFrame]);

  CHECK_VK_RESULT_FATAL(graphicsQueue.handle.submit(
                            1, &submitInfo, Swapchain::GetCurrentFrameFence()),
                        "Failed to submit draw command buffer.");

  result = Swapchain::Present(&renderFinishedSemaphores[currentFrame]);
  if (result == vk::Result::eErrorOutOfDateKHR ||
      result == vk::Result::eSuboptimalKHR)
    renderContext.refresh();
  else
    CHECK_VK_RESULT_FATAL(result, "Failed to present Swapchain image.");
}

//-----------------------------------------------------------------------------
// INITIALIZATION
//-----------------------------------------------------------------------------

static void CreateInstance() {
  vk::ApplicationInfo appInfo("Renderer", VK_MAKE_VERSION(1, 0, 0), "No Engine",
                              VK_MAKE_VERSION(1, 0, 0), VK_API_VERSION_1_1);

  vk::InstanceCreateInfo createInfo;
  createInfo.pApplicationInfo = &appInfo;

  std::vector<const char *> extensions(g_requiredInstanceExtensions.data(),
                                       g_requiredInstanceExtensions.data() +
                                           g_requiredInstanceExtensions.size());

  extensions.insert(extensions.end(), g_instanceExtensions.begin(),
                    g_instanceExtensions.end());

  createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
  createInfo.ppEnabledExtensionNames = extensions.data();

#ifndef NDEBUG
  createInfo.enabledLayerCount =
      static_cast<uint32_t>(g_validationLayers.size());
  createInfo.ppEnabledLayerNames = g_validationLayers.data();
#else
  createInfo.enabledLayerCount = 0;
#endif

  CHECK_VK_RESULT_FATAL(
      vk::createInstance(&createInfo, g_allocationCallbacks, &g_instance),
      "Failed to init vulkan instance");

  g_dldy.init(g_instance, vkGetInstanceProcAddr);

#ifndef NDEBUG
  for (size_t i = 0; i < g_debugMessengersInfos.size(); ++i)
    g_debugMessengers.push_back(g_instance.createDebugUtilsMessengerEXT(
        g_debugMessengersInfos[i].MakeCreateInfo(), g_allocationCallbacks,
        g_dldy));
#endif
}

static void InitSurface(void *windowHandle) {
#ifdef _WIN32
  vk::Win32SurfaceCreateInfoKHR createInfo{vk::Win32SurfaceCreateFlagsKHR(),
                                           GetModuleHandle(nullptr),
                                           (HWND)windowHandle};

  CHECK_VK_RESULT_FATAL(g_instance.createWin32SurfaceKHR(&createInfo,
                                                         g_allocationCallbacks,
                                                         &g_surface, g_dldy),
                        "Failed to create window surface");
#endif

  // TODO Support other OS
}

static void InitDevice() {
  const std::vector<vk::PhysicalDevice> devices =
      g_instance.enumeratePhysicalDevices();

  if (devices.empty())
    throw std::runtime_error("failed to find GPUs with Vulkan support!");

  {
    // Use an ordered map to automatically sort candidates by increasing score
    std::multimap<uint64_t, vk::PhysicalDevice> candidates;

    for (const auto &device : devices) {
      uint64_t score = RateDeviceSuitability(device, g_surface);
      candidates.insert(std::make_pair(score, device));
    }

    // Check if the best candidate is suitable at all
    if (candidates.rbegin()->first > 0)
      g_physicalDevice = candidates.rbegin()->second;
    else
      throw std::runtime_error("Failed to find a suitable GPU");
  }

  std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
  const QueueFamilyIndices indices =
      GetQueueFamilies(g_physicalDevice, g_surface);
  const std::set<int> uniqueQueueFamilies = {
      indices.graphicsFamily, indices.presentFamily, indices.transferFamily};

  // TODO Manage priorities if using different queues from the same family
  float queuePriority = 1.0f;
  for (int queueFamily : uniqueQueueFamilies) {
    vk::DeviceQueueCreateInfo queueCreateInfo(vk::DeviceQueueCreateFlags(),
                                              queueFamily, 1, &queuePriority);

    queueCreateInfos.push_back(queueCreateInfo);
  }

  vk::PhysicalDeviceFeatures deviceFeatures = {};

  vk::DeviceCreateInfo createInfo(
      vk::DeviceCreateFlags(), static_cast<uint32_t>(queueCreateInfos.size()),
      queueCreateInfos.data(),
#ifndef NDEBUG
      static_cast<uint32_t>(g_validationLayers.size()),
      g_validationLayers.data(),
#else
      0, nullptr,
#endif
      static_cast<uint32_t>(g_deviceExtensions.size()),
      g_deviceExtensions.data(), &deviceFeatures);

  CHECK_VK_RESULT_FATAL(g_physicalDevice.createDevice(
                            &createInfo, g_allocationCallbacks, &g_device),
                        "Failed to create logical device");

  g_dldy.init(g_instance, vkGetInstanceProcAddr, g_device, vkGetDeviceProcAddr);

  // TODO Different index for each (need to create a queue for each then)?
  graphicsQueue.index = 0;
  graphicsQueue.handle =
      g_device.getQueue(indices.graphicsFamily, graphicsQueue.index);

  Queue presentQueue;
  presentQueue.index = 0;
  presentQueue.handle =
      g_device.getQueue(indices.presentFamily, presentQueue.index);

  Queue transferQueue;
  transferQueue.index = 0;
  transferQueue.handle =
      g_device.getQueue(indices.transferFamily, transferQueue.index);

  Swapchain::SetPresentQueue(presentQueue);
  Allocator::SetTransferQueue(transferQueue);

  vk::SemaphoreCreateInfo semaphoreInfo = {};

  for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++)
    CHECK_VK_RESULT_FATAL(
        g_device.createSemaphore(&semaphoreInfo, g_allocationCallbacks,
                                 &renderFinishedSemaphores[i]),
        "Failed to create render finished semaphore.");
}

static void InitCommandPool() {
  QueueFamilyIndices queueFamilyIndices =
      GetQueueFamilies(g_physicalDevice, g_surface);

  vk::CommandPoolCreateInfo poolInfo(
      vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
      queueFamilyIndices.graphicsFamily);

  CHECK_VK_RESULT_FATAL(g_device.createCommandPool(
                            &poolInfo, g_allocationCallbacks, &g_commandPool),
                        "Failed to create command pool.");
}

static void InitVulkan(void *windowHandle) {
  CreateInstance();
  InitSurface(windowHandle);
  // CHECK_VK_RESULT_FATAL((vk::Result)WindowHandler::CreateSurface(
  //                          (VkInstance)g_instance,
  //                          (VkAllocationCallbacks *)g_allocationCallbacks,
  //                          (VkSurfaceKHR *)&g_surface),
  //                      "Failed to create window surface");
  InitDevice();

  InitCommandPool();
  Allocator::Init();

  renderContext.init();

#ifndef NDEBUG
  // SetMainObjectsDebugNames(); //Crash with newest drivers
#endif
}

//-----------------------------------------------------------------------------
// USER FUNCTIONS
//-----------------------------------------------------------------------------

void Init(unsigned int width, unsigned int height, void *windowHandle) {
  assert(width != 0 && height != 0);
  requestedWidth = width, requestedHeight = height;

  TRY_CATCH_BLOCK("Failed to init renderer",

                  InitVulkan(windowHandle););
}

void Update() {
  TRY_CATCH_BLOCK("Failed to update renderer",

                  Draw(););
}

void Shutdown() {
  g_device.waitIdle();

  TRY_CATCH_BLOCK(
      "Failed to shutdown renderer",

      for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++)
          g_device.destroySemaphore(renderFinishedSemaphores[i],
                                    g_allocationCallbacks);

      renderContext.destroy();

      // g_device.destroyDescriptorSetLayout(g_baseDescriptorSetLayout,
      // g_allocationCallbacks);

      // for (size_t i = 0; i < g_globalDSL.size(); ++i)
      //    g_device.destroyDescriptorSetLayout(g_globalDSL[i],
      //    g_allocationCallbacks);

      Allocator::Destroy();

      g_device.destroyCommandPool(g_commandPool, g_allocationCallbacks);

      g_device.destroy(g_allocationCallbacks);

#ifndef NDEBUG
      for (size_t i = 0; i < g_debugMessengers.size(); ++i)
          g_instance.destroyDebugUtilsMessengerEXT(
              g_debugMessengers[i], g_allocationCallbacks, g_dldy);
#endif

      g_instance.destroySurfaceKHR(g_surface, g_allocationCallbacks);
      g_instance.destroy(g_allocationCallbacks););
}

void Resize(unsigned int width, unsigned int height) {
  requestedWidth = width, requestedHeight = height;
  renderContext.refresh();
}

void SetPresentMode(PresentMode presentMode) {
  Swapchain::PreferredPresentMode =
      static_cast<vk::PresentModeKHR>(presentMode);
  renderContext.refresh();
}

void GetCurrentResolution(int &w, int &h) {
  vk::Extent2D extent = Swapchain::GetExtent();

  w = extent.width;
  h = extent.height;
}

uint64_t CreateModelFromPrimitive(EPrimitive primitive) {
  Model *model = new Model();
  switch (primitive) {
  case EPrimitive::Plane:
    model->init_from_primitive<Plane>();
    break;
  case EPrimitive::Cube:
    model->init_from_primitive<Cube>();
    break;
  }

  renderContext.models.push_back(model);

  return uintptr_t(renderContext.models.back());
}

uint64_t CreateModelFromObj(const std::string &objFilename) {
  Model *model = new Model();

  model->init_from_obj_file(objFilename);

  renderContext.models.push_back(model);

  return uintptr_t(renderContext.models.back());
}

ModelInstance *Spawn(uint64_t modelId, Vec3 pos, Vec3 rot, Vec3 scale) {
  return reinterpret_cast<Model *>(modelId)->spawn_instance(pos, rot, scale);
}

void Destroy(ModelInstance *instance) {
  reinterpret_cast<Model *>(instance->get_model_id())
      ->destroy_instance(instance);
}

Light *SpawnLight(Vec3 position, Vec3 color, float ambientStrength) {
  return renderContext.spawn_light(position, color, ambientStrength);
}

} // namespace Renderer
