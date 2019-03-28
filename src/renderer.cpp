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
  std::vector<vk::Framebuffer> presentFramebuffers;
  vk::Framebuffer geomFramebuffer;

  vk::RenderPass geomRenderPass;
  vk::RenderPass lightRenderPass;

  Attachment depthAttachment;
  Attachment posAttachment;
  Attachment normalAttachment;
  Attachment colorAttachment;

  static constexpr uint32_t depthAttachmentArrayIndex = DEPTH_BUFFER_INDEX;

  static constexpr uint32_t posAttachmentArrayIndex = POS_BUFFER_INDEX;
  static constexpr uint32_t normalAttachmentArrayIndex = NORMAL_BUFFER_INDEX;
  static constexpr uint32_t colorAttachmentArrayIndex = COLOR_BUFFER_INDEX;

  static constexpr uint32_t gPassAttachmentCount = G_BUFFER_SIZE;

  Shader gShader;
  Shader lightShader;
  Shader overlayShader;

  std::vector<Model *> models;
  std::vector<Light *> lights;

  Pass ssaoPass;
  Pass blurPass;

  void init_ssao_pass() {
    vk::Extent2D extent = Swapchain::GetExtent();

    AttachmentInfo attachmentInfo;
    attachmentInfo.extent = extent;
    attachmentInfo.format = vk::Format::eR32Sfloat;
    attachmentInfo.isDepth = false;

    std::vector<const Image *> images;
    images.resize(G_BUFFER_SIZE);

    images[posAttachmentArrayIndex] = &posAttachment.get_image();
    images[normalAttachmentArrayIndex] = &normalAttachment.get_image();
    images[colorAttachmentArrayIndex] = &colorAttachment.get_image();
    images[depthAttachmentArrayIndex] = &depthAttachment.get_image();

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
    shaderInfo.vertPath = "../resources/shaders/spv/ssao.vert.spv";
    shaderInfo.fragPath = "../resources/shaders/spv/ssao.frag.spv";
    shaderInfo.useVertexInput = false;
    shaderInfo.cull = false;
    shaderInfo.blendEnable = false;
    shaderInfo.drawModels = false;
    shaderInfo.pushConstants = pushConstantRanges;
    shaderInfo.descriptors = descriptorSetInfos;
    shaderInfo.ubos = ubos;

    ssaoPass.init(extent, {attachmentInfo}, {shaderInfo});
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
    shaderInfo.vertPath = "../resources/shaders/spv/blur.vert.spv";
    shaderInfo.fragPath = "../resources/shaders/spv/blur.frag.spv";
    shaderInfo.useVertexInput = false;
    shaderInfo.cull = false;
    shaderInfo.blendEnable = false;
    shaderInfo.drawModels = false;
    shaderInfo.descriptors = {descriptorSetInfo};

    blurPass.init(extent, {attachmentInfo}, {shaderInfo});
  }

  void destroy_ssao_pass() { ssaoPass.destroy(); }

  void destroy_blur_pass() { blurPass.destroy(); }

  void init_render_passes() {
    {
      // First Pass
      std::array<vk::AttachmentDescription, gPassAttachmentCount>
          attachmentDescriptions = {};

      depthAttachment.requiredFormat = FindSupportedFormat(
          g_physicalDevice,
          {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint,
           vk::Format::eD24UnormS8Uint},
          vk::ImageTiling::eOptimal,
          vk::FormatFeatureFlagBits::eDepthStencilAttachment);

      attachmentDescriptions[depthAttachmentArrayIndex] =
          depthAttachment.make_attachment_description(
              vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal,
              vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal);

      posAttachment.requiredFormat = vk::Format::eR32G32B32A32Sfloat;

      attachmentDescriptions[posAttachmentArrayIndex] =
          posAttachment.make_attachment_description(
              vk::ImageLayout::eShaderReadOnlyOptimal,
              vk::ImageLayout::eShaderReadOnlyOptimal);

      normalAttachment.requiredFormat = vk::Format::eR32G32B32A32Sfloat;

      attachmentDescriptions[normalAttachmentArrayIndex] =
          normalAttachment.make_attachment_description(
              vk::ImageLayout::eShaderReadOnlyOptimal,
              vk::ImageLayout::eShaderReadOnlyOptimal);

      colorAttachment.requiredFormat = vk::Format::eR32G32B32A32Sfloat;

      attachmentDescriptions[colorAttachmentArrayIndex] =
          colorAttachment.make_attachment_description(
              vk::ImageLayout::eShaderReadOnlyOptimal,
              vk::ImageLayout::eShaderReadOnlyOptimal);

      // Color Attachments
      std::array<vk::AttachmentReference, gPassAttachmentCount - 1> colorRefs;

      colorRefs[posAttachmentArrayIndex - 1] = vk::AttachmentReference(
          posAttachmentArrayIndex, vk::ImageLayout::eColorAttachmentOptimal);

      colorRefs[normalAttachmentArrayIndex - 1] = vk::AttachmentReference(
          normalAttachmentArrayIndex, vk::ImageLayout::eColorAttachmentOptimal);

      colorRefs[colorAttachmentArrayIndex - 1] = vk::AttachmentReference(
          colorAttachmentArrayIndex, vk::ImageLayout::eColorAttachmentOptimal);

      vk::AttachmentReference depthAttachmentRef(
          depthAttachmentArrayIndex,
          vk::ImageLayout::eDepthStencilAttachmentOptimal);

      vk::SubpassDescription subpass(
          vk::SubpassDescriptionFlags(), vk::PipelineBindPoint::eGraphics, 0,
          nullptr, static_cast<uint32_t>(colorRefs.size()), colorRefs.data(),
          nullptr, &depthAttachmentRef);

      vk::RenderPassCreateInfo renderPassInfo(
          vk::RenderPassCreateFlags(),
          static_cast<uint32_t>(attachmentDescriptions.size()),
          attachmentDescriptions.data(), 1, &subpass, 0, nullptr);

      CHECK_VK_RESULT_FATAL(g_device.createRenderPass(&renderPassInfo,
                                                      g_allocationCallbacks,
                                                      &geomRenderPass),
                            "Failed to create render pass.");

#ifndef NDEBUG
      g_device.setDebugUtilsObjectNameEXT(
          {vk::ObjectType::eRenderPass,
           (uint64_t)((VkRenderPass)geomRenderPass), "Geometry Render Pass"},
          g_dldy);
#endif
    }

    {
      // Fourth Pass
      std::array<vk::AttachmentDescription, 1> attachmentDescriptions = {
          Swapchain::GetAttachmentDescription()};

      vk::AttachmentReference swapchainColorAttachmentRef(
          0, vk::ImageLayout::eColorAttachmentOptimal);

      vk::SubpassDescription subpass(vk::SubpassDescriptionFlags(),
                                     vk::PipelineBindPoint::eGraphics, 0,
                                     nullptr, 1, &swapchainColorAttachmentRef);

      // vk::SubpassDependency dependency(
      //    VK_SUBPASS_EXTERNAL, 0,
      //    vk::PipelineStageFlagBits::eColorAttachmentOutput,
      //    vk::PipelineStageFlagBits::eColorAttachmentOutput,
      //    (vk::AccessFlagBits)0,
      //    vk::AccessFlagBits::eColorAttachmentRead |
      //        vk::AccessFlagBits::eColorAttachmentWrite);

      vk::RenderPassCreateInfo renderPassInfo(
          vk::RenderPassCreateFlags(),
          static_cast<uint32_t>(attachmentDescriptions.size()),
          attachmentDescriptions.data(), 1, &subpass, 0, nullptr);

      CHECK_VK_RESULT_FATAL(g_device.createRenderPass(&renderPassInfo,
                                                      g_allocationCallbacks,
                                                      &lightRenderPass),
                            "Failed to create render pass.");

#ifndef NDEBUG
      g_device.setDebugUtilsObjectNameEXT(
          {vk::ObjectType::eRenderPass,
           (uint64_t)((VkRenderPass)lightRenderPass), "Light Render Pass"},
          g_dldy);
#endif
    }
  }

  void destroy_render_pass() {
    g_device.destroyRenderPass(geomRenderPass, g_allocationCallbacks);
    g_device.destroyRenderPass(lightRenderPass, g_allocationCallbacks);
  }

  void init_shaders() {

    std::vector<vk::PushConstantRange> pushConstantRanges;

    PushConstantDescriptor<ModelInstancePushConstant>
        modelInstancePushConstantRange{};
    pushConstantRanges.push_back(
        modelInstancePushConstantRange.make_push_constant_range());

    std::vector<UniformBufferObject *> ubos;
    ubos.push_back(&CommonResources::CameraUBOSet);

    gShader.init("../resources/shaders/spv/geom.vert.spv",
                 "../resources/shaders/spv/geom.frag.spv", geomRenderPass, 3,
                 true, true, false, pushConstantRanges, {}, ubos);

    std::vector<const Image *> images;
    images.resize(G_BUFFER_SIZE);

    images[posAttachmentArrayIndex] = &posAttachment.get_image();
    images[normalAttachmentArrayIndex] = &normalAttachment.get_image();
    images[colorAttachmentArrayIndex] = &colorAttachment.get_image();
    images[depthAttachmentArrayIndex] = &depthAttachment.get_image();

    std::vector<const vk::Sampler *> samplers(G_BUFFER_SIZE,
                                              &CommonResources::BaseSampler);

    pushConstantRanges.clear();
    PushConstantDescriptor<LightPassPushConstant> lightPassPushConstantRange{};
    pushConstantRanges.push_back(
        lightPassPushConstantRange.make_push_constant_range());

    ubos.push_back(&CommonResources::LightUBOSet);

    std::vector<DescriptorSetInfo> dsInfo;

    dsInfo.push_back({CommonResources::GBufferLayout, {}, images, samplers});
    dsInfo.push_back({CommonResources::UniqueTextureLayout,
                      {},
                      {&blurPass.get_attachment_image()},
                      {&CommonResources::BaseSampler}});
    dsInfo.push_back({CommonResources::UniqueTextureLayout,
                      {},
                      {&CommonResources::DitherTex},
                      {&CommonResources::RepeatSampler}});

    lightShader.init("../resources/shaders/spv/light.vert.spv",
                     "../resources/shaders/spv/light.frag.spv", lightRenderPass,
                     1, false, false, true, pushConstantRanges, dsInfo, ubos);

    pushConstantRanges.clear();

    PushConstantDescriptor<ExtentPushConstant> extentPushConstantRange{};
    pushConstantRanges.push_back(
        extentPushConstantRange.make_push_constant_range());

    dsInfo.clear();
    dsInfo.push_back({CommonResources::GBufferLayout, {}, images, samplers});

    overlayShader.init("../resources/shaders/spv/overlay.vert.spv",
                       "../resources/shaders/spv/overlay.frag.spv",
                       lightRenderPass, 1, false, false, true,
                       pushConstantRanges, dsInfo);
  }

  void destroy_shaders() {
    gShader.destroy();
    lightShader.destroy();
    overlayShader.destroy();
  }

  void init_framebuffers() {
    vk::Extent2D extent = Swapchain::GetExtent();

    {
      depthAttachment.init(
          Swapchain::GetExtent().width, Swapchain::GetExtent().height,
          vk::ImageTiling::eOptimal,
          vk::ImageUsageFlagBits::eDepthStencilAttachment |
              vk::ImageUsageFlagBits::eSampled,
          vk::MemoryPropertyFlagBits::eDeviceLocal,
          vk::ImageAspectFlagBits::eDepth,
          vk::ImageLayout::eDepthAttachmentStencilReadOnlyOptimal);

      posAttachment.init(Swapchain::GetExtent().width,
                         Swapchain::GetExtent().height);

      normalAttachment.init(Swapchain::GetExtent().width,
                            Swapchain::GetExtent().height);

      colorAttachment.init(Swapchain::GetExtent().width,
                           Swapchain::GetExtent().height);

      std::array<vk::ImageView, gPassAttachmentCount> attachments;
      attachments[depthAttachmentArrayIndex] = depthAttachment.get_image_view();
      attachments[posAttachmentArrayIndex] = posAttachment.get_image_view();
      attachments[normalAttachmentArrayIndex] =
          normalAttachment.get_image_view();
      attachments[colorAttachmentArrayIndex] = colorAttachment.get_image_view();

      vk::FramebufferCreateInfo framebufferInfo(
          vk::FramebufferCreateFlags(), geomRenderPass,
          static_cast<uint32_t>(attachments.size()), attachments.data(),
          extent.width, extent.height, 1);

      CHECK_VK_RESULT_FATAL(g_device.createFramebuffer(&framebufferInfo,
                                                       g_allocationCallbacks,
                                                       &geomFramebuffer),
                            "Failed to create framebuffer.");
    }

    {
      const std::vector<vk::ImageView> &attachments =
          Swapchain::GetImageViews();

      presentFramebuffers.resize(Swapchain::ImageCount());
      for (size_t i = 0; i < presentFramebuffers.size(); i++) {
        std::array<vk::ImageView, 1> iattachments = {attachments[i]};

        vk::FramebufferCreateInfo framebufferInfo(
            vk::FramebufferCreateFlags(), lightRenderPass,
            static_cast<uint32_t>(iattachments.size()), iattachments.data(),
            extent.width, extent.height, 1);

        CHECK_VK_RESULT_FATAL(
            g_device.createFramebuffer(&framebufferInfo, g_allocationCallbacks,
                                       &presentFramebuffers[i]),
            "Failed to create framebuffer.");
      }
    }
  }

  void destroy_framebuffers() {
    for (size_t i = 0; i < presentFramebuffers.size(); ++i)
      g_device.destroyFramebuffer(presentFramebuffers[i],
                                  g_allocationCallbacks);

    g_device.destroyFramebuffer(geomFramebuffer, g_allocationCallbacks);

    depthAttachment.destroy();
    posAttachment.destroy();
    normalAttachment.destroy();
    colorAttachment.destroy();
  }

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

    static constexpr const std::array<float, 4> clearColorArray = {
        {0.0f, 0.0f, 0.0f, 1.0f}};

    vk::ClearColorValue clearColorValue = clearColorArray;
    vk::ClearDepthStencilValue clearDepthValue = {1.0f};

    std::array<vk::ClearValue, 4> clearColors = {
        clearDepthValue, clearColorValue, clearColorValue, clearColorValue};

    CHECK_VK_RESULT_FATAL(commandbuffers[index].begin(&beginInfo),
                          "Failed to begin recording command buffer.");

    vk::RenderPassBeginInfo renderPassInfo(
        geomRenderPass, geomFramebuffer, {{0, 0}, extent},
        static_cast<uint32_t>(clearColors.size()), clearColors.data());

    commandbuffers[index].setViewport(
        0, 1,
        &viewport); // TODO buffers are recorded once so can't change viewport

    commandbuffers[index].setScissor(
        0, 1,
        &scissor); // TODO buffers are recorded once so can't change scissor

    commandbuffers[index].beginRenderPass(&renderPassInfo,
                                          vk::SubpassContents::eInline);

    gShader.bind_pipeline(commandbuffers[index]);

    CameraUBO cam{};
    cam.view = Camera::ViewMatrix;
    cam.proj = Camera::ProjMatrix;
    cam.viewPos = Maths::EigenizeVec3(Camera::Position);

    CommonResources::CameraUBOSet.write(index, &cam, sizeof(cam), 0);

    commandbuffers[index].bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, gShader.get_pipeline_layout(), 0, 1,
        &CommonResources::CameraUBOSet.get_descriptor_set(index), 0, nullptr);

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

    for (size_t j = 0; j < models.size(); ++j)
      models[j]->record(commandbuffers[index], gShader, index);

    commandbuffers[index].endRenderPass();

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
                      whiteClearValue);
      blurPass.record(commandbuffers[index], index, {}, whiteClearValue);

    } else
      blurPass.record_clear_attachments(commandbuffers[index], whiteClearValue);

    // light pass
    vk::ClearValue clearValue = (vk::ClearValue)clearColorValue;

    vk::RenderPassBeginInfo lightRenderPassInfo(
        lightRenderPass, presentFramebuffers[index], {{0, 0}, extent}, 1,
        &clearValue);

    commandbuffers[index].beginRenderPass(&lightRenderPassInfo,
                                          vk::SubpassContents::eInline);

    lightShader.bind_pipeline(commandbuffers[index]);

    commandbuffers[index].bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, lightShader.get_pipeline_layout(), 0,
        1, &CommonResources::CameraUBOSet.get_descriptor_set(index), 0,
        nullptr);

    commandbuffers[index].bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, lightShader.get_pipeline_layout(), 1,
        1, &CommonResources::LightUBOSet.get_descriptor_set(index), 0, nullptr);

    lightShader.bind_descriptors(commandbuffers[index], 2);

    LightPassPushConstant lightPassPushConstant{};
    lightPassPushConstant.numLights = lights.size();

    commandbuffers[index].pushConstants(
        lightShader.get_pipeline_layout(),
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
        0, sizeof(lightPassPushConstant), &lightPassPushConstant);

    // Draw scene into texture
    commandbuffers[index].draw(6, 1, 0, 0);

    if (Config::ShowGBuffer) {
      overlayShader.bind_pipeline(commandbuffers[index]);
      overlayShader.bind_descriptors(commandbuffers[index]);

      ExtentPushConstant ePc = {};
      ePc.xExtent = extent.width;
      ePc.yExtent = extent.height;
      ePc.ratio = 1.f / 6.f;

      commandbuffers[index].pushConstants(
          overlayShader.get_pipeline_layout(),
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
          0, sizeof(ePc), &ePc);

      // Draw G-Buffer overlay
      commandbuffers[index].draw(6 * 4, 1, 0, 0);
    }

    commandbuffers[index].endRenderPass();

    commandbuffers[index]
        .end(); // Strangely, it does returns void instead of vk::Result so no
                // error checking is possible here
  }

  void init() {
    Swapchain::Init(requestedWidth, requestedHeight);

    CommonResources::InitDescriptorLayouts();
    CommonResources::InitUniformBufferObjects();

    init_render_passes();

    init_framebuffers();

    // Resources for descriptors
    CommonResources::InitSamplers();
    CommonResources::InitPostProcessResources();

    init_ssao_pass();
    init_blur_pass();

    init_shaders();

    init_commandbuffers();
  }

  void destroy(bool freeModels = true) {

    CommonResources::DestroyDescriptorLayouts();
    CommonResources::DestroyUniformBufferObjects();

    CommonResources::DestroySamplers();
    CommonResources::DestroyPostProcessResources();

    destroy_framebuffers();
    destroy_commandbuffers();

    destroy_blur_pass();
    destroy_ssao_pass();

    destroy_shaders();
    destroy_render_pass();

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

    destroy_framebuffers();
    destroy_commandbuffers();

    destroy_blur_pass();
    destroy_ssao_pass();

    destroy_shaders();

    Swapchain::Destroy();
    Swapchain::Init(requestedWidth, requestedHeight);

    init_framebuffers();
    init_commandbuffers();

    init_ssao_pass();
    init_blur_pass();

    init_shaders();

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

  g_dldy.init(g_instance);

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
    std::multimap<int, vk::PhysicalDevice> candidates;

    for (const auto &device : devices) {
      int score = RateDeviceSuitability(device, g_surface);
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

  g_dldy.init(g_instance, g_device);

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
