#include "renderer.hpp"

#include "configuration_helper.hpp"
#include "data_structures.hpp"
#include "maths.hpp"
#include "memory.hpp"
#include "window_handler.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include <map>
#include <unordered_map>
#include <vector>

namespace Renderer {
//-----------------------------------------------------------------------------
// GLOBAL CONTEXT
//-----------------------------------------------------------------------------

static vk::Instance g_instance;
static vk::PhysicalDevice g_physicalDevice;

static vk::Device g_device;

static vk::SurfaceKHR g_surface;

static vk::Format g_requiredFormat;
static vk::Extent2D g_extent;

struct Queue {
  int index = -1;
  vk::Queue handle = nullptr;
};

static Queue g_graphicsQueue;
static Queue g_presentQueue;

static vk::CommandPool g_commandPool;

// static vk::DescriptorSetLayout g_baseDescriptorSetLayout;

static vk::RenderPass g_renderPass;

static vk::AllocationCallbacks *g_allocator = nullptr;

static vk::DispatchLoaderDynamic g_dldy;

static bool g_framebufferResized = false;

#ifndef NDEBUG
static std::vector<vk::DebugUtilsMessengerEXT> g_debugMessengers;
#endif

//-----------------------------------------------------------------------------
// RENDER CONTEXT
//-----------------------------------------------------------------------------

static struct {
private:
  vk::SwapchainKHR handle;

  std::vector<vk::Image> images;
  std::vector<vk::ImageView> imageViews;

public:
  vk::PresentModeKHR requiredPresentMode = vk::PresentModeKHR::eMailbox;

  const vk::SwapchainKHR &get_handle() const { return handle; }

  size_t image_count() const { return images.size(); }
  const vk::ImageView get_image_view(size_t index) const {
    return imageViews[index];
  }

  void init() {
    SwapChainSupportDetails swapChainSupport =
        QuerySwapChainSupport(g_physicalDevice, g_surface);

    vk::SurfaceFormatKHR format =
        swapChainSupport.formats[0]; // safe since it has been checked that
                                     // there are available formats
    if (swapChainSupport.formats.size() == 1 &&
        swapChainSupport.formats[0].format == vk::Format::eUndefined)
      format = {vk::Format::eB8G8R8A8Unorm, vk::ColorSpaceKHR::eSrgbNonlinear};
    else {
      for (const auto &availableFormat : swapChainSupport.formats) {
        if (availableFormat.format == vk::Format::eB8G8R8A8Unorm &&
            availableFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear)
          format = availableFormat;
      }
    }

    vk::PresentModeKHR presentMode =
        vk::PresentModeKHR::eFifo; // Fifo mode's availability is guaranteed
    if (requiredPresentMode != vk::PresentModeKHR::eFifo) {
      for (const auto &availablePresentMode : swapChainSupport.presentModes) {
        if (availablePresentMode == requiredPresentMode) {
          presentMode = availablePresentMode;
          break;
        }
      }

      if (presentMode == vk::PresentModeKHR::eFifo) {
        requiredPresentMode = vk::PresentModeKHR::eFifo;
        std::cout << vk::to_string(requiredPresentMode)
                  << " not supported. Using FIFO V-Sync mode instead."
                  << std::endl;
      }
    }

    if (swapChainSupport.capabilities.currentExtent.width !=
        std::numeric_limits<uint32_t>::max()) {
      g_extent = swapChainSupport.capabilities.currentExtent;
    } else {
      int width, height;
      WindowHandler::GetFramebufferSize(&width, &height);

      VkExtent2D actualExtent = {width, height};

      actualExtent.width =
          std::max(swapChainSupport.capabilities.minImageExtent.width,
                   std::min(swapChainSupport.capabilities.maxImageExtent.width,
                            actualExtent.width));
      actualExtent.height =
          std::max(swapChainSupport.capabilities.minImageExtent.height,
                   std::min(swapChainSupport.capabilities.maxImageExtent.height,
                            actualExtent.height));

      g_extent = actualExtent;
    }

    uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
    if (swapChainSupport.capabilities.maxImageCount > 0 &&
        imageCount > swapChainSupport.capabilities.maxImageCount)
      imageCount = swapChainSupport.capabilities.maxImageCount;

    QueueFamilyIndices indices = GetQueueFamilies(g_physicalDevice, g_surface);
    uint32_t queueFamilyIndices[] = {(uint32_t)indices.graphicsFamily,
                                     (uint32_t)indices.presentFamily};

    vk::SharingMode imageSharingMode = vk::SharingMode::eExclusive;
    uint32_t queueFamilyIndexCount = 0;

    if (indices.graphicsFamily != indices.presentFamily) {
      imageSharingMode = vk::SharingMode::eConcurrent;
      queueFamilyIndexCount = sizeof(queueFamilyIndices) / sizeof(uint32_t);
    }

    vk::SwapchainCreateInfoKHR createInfo(
        vk::SwapchainCreateFlagsKHR(), g_surface, imageCount, format.format,
        format.colorSpace, g_extent, 1,
        vk::ImageUsageFlagBits::eColorAttachment, imageSharingMode,
        queueFamilyIndexCount, queueFamilyIndices,
        swapChainSupport.capabilities.currentTransform,
        vk::CompositeAlphaFlagBitsKHR::eOpaque, presentMode, VK_TRUE
        // swapchain
    );

    g_device.createSwapchainKHR(&createInfo, g_allocator, &handle);

    images = g_device.getSwapchainImagesKHR(handle);
    g_requiredFormat = format.format;

    vk::ImageSubresourceRange imageSubresourceRange(
        vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);

    imageViews.resize(images.size());
    for (size_t i = 0; i < images.size(); i++) {
      vk::ImageViewCreateInfo createInfo(
          vk::ImageViewCreateFlags(), images[i], vk::ImageViewType::e2D,
          g_requiredFormat, vk::ComponentMapping(), imageSubresourceRange);

      CHECK_VK_RESULT_FATAL(
          g_device.createImageView(&createInfo, g_allocator, &imageViews[i]),
          "Failed to create image views");
    }
  }

  void destroy() {
    for (auto imageView : imageViews)
      g_device.destroyImageView(imageView, g_allocator);

    g_device.destroySwapchainKHR(handle, g_allocator);
  }
} g_swapchain;

struct Shader {
private:
  // vk::DescriptorSetLayout descriptorSetLayout;

  vk::Pipeline pipeline;
  vk::PipelineLayout pipelineLayout;

  static vk::ShaderModule CreateShaderModule(const std::vector<char> &code) {
    vk::ShaderModuleCreateInfo createInfo(
        vk::ShaderModuleCreateFlags(), code.size(),
        reinterpret_cast<const uint32_t *>(code.data()));

    vk::ShaderModule shaderModule;
    CHECK_VK_RESULT_FATAL(
        g_device.createShaderModule(&createInfo, g_allocator, &shaderModule),
        "Failed to create shader module");

    return shaderModule;
  }

  void init_pipeline(const std::string &vertPath, const std::string &fragPath) {
    auto vertShaderCode = ReadFile(vertPath);
    auto fragShaderCode = ReadFile(fragPath);

    vk::ShaderModule vertShaderModule = CreateShaderModule(vertShaderCode);
    vk::ShaderModule fragShaderModule = CreateShaderModule(fragShaderCode);

    vk::PipelineShaderStageCreateInfo vertShaderStageInfo(
        vk::PipelineShaderStageCreateFlags(), vk::ShaderStageFlagBits::eVertex,
        vertShaderModule, "main");

    vk::PipelineShaderStageCreateInfo fragShaderStageInfo(
        vk::PipelineShaderStageCreateFlags(),
        vk::ShaderStageFlagBits::eFragment, fragShaderModule, "main");

    vk::PipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo,
                                                        fragShaderStageInfo};

    vk::VertexInputBindingDescription bindingDescription =
        Vertex<3>::GetBindingDescription();
    auto attributeDescriptions = Vertex<3>::GetAttributeDescription();

    vk::PipelineVertexInputStateCreateInfo vertexInputInfo(
        vk::PipelineVertexInputStateCreateFlags(), 1, &bindingDescription,
        static_cast<uint32_t>(attributeDescriptions.size()),
        reinterpret_cast<vk::VertexInputAttributeDescription *>(
            attributeDescriptions.data()));

    vk::PipelineInputAssemblyStateCreateInfo inputAssembly(
        vk::PipelineInputAssemblyStateCreateFlags(),
        vk::PrimitiveTopology::eTriangleList, VK_FALSE);

    vk::Viewport viewport(0, 0, (float)g_extent.width, (float)g_extent.height,
                          0, 1);

    vk::Rect2D scissor({0, 0}, g_extent);

    vk::PipelineViewportStateCreateInfo viewportState(
        vk::PipelineViewportStateCreateFlags(), 1, &viewport, 1, &scissor);

    vk::PipelineRasterizationStateCreateInfo rasterizer(
        vk::PipelineRasterizationStateCreateFlags(), VK_FALSE, VK_FALSE,
        vk::PolygonMode::eFill, vk::CullModeFlagBits::eBack,
        vk::FrontFace::eCounterClockwise, VK_FALSE, 0, 0, 0, 1);

    vk::PipelineMultisampleStateCreateInfo multisampling(
        vk::PipelineMultisampleStateCreateFlags(), vk::SampleCountFlagBits::e1,
        VK_FALSE, 1, nullptr, VK_FALSE, VK_FALSE);

    vk::PipelineColorBlendAttachmentState colorBlendAttachment(
        VK_TRUE, vk::BlendFactor::eSrcAlpha, vk::BlendFactor::eOneMinusSrcAlpha,
        vk::BlendOp::eAdd, vk::BlendFactor::eOne, vk::BlendFactor::eZero,
        vk::BlendOp::eAdd,
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

    vk::PipelineColorBlendStateCreateInfo colorBlending(
        vk::PipelineColorBlendStateCreateFlags(), VK_FALSE, vk::LogicOp::eCopy,
        1, &colorBlendAttachment);

    vk::DynamicState dynamicState = vk::DynamicState::eViewport;
    vk::PipelineDynamicStateCreateInfo dynamicStateCreateInfo(
        vk::PipelineDynamicStateCreateFlags(), 1, &dynamicState);

    vk::PushConstantRange pushConstantRange{vk::ShaderStageFlagBits::eVertex, 0,
                                            sizeof(Vertex<3>) +
                                                sizeof(Eigen::Matrix4f) * 2};

    // vk::PipelineLayoutCreateInfo pipelineLayoutInfo(
    //    vk::PipelineLayoutCreateFlags(), 1, &g_baseDescriptorSetLayout, 1,
    //    &pushConstantRange); // TODO add custom shader ubos

    vk::PipelineLayoutCreateInfo pipelineLayoutInfo(
        vk::PipelineLayoutCreateFlags(), 0, nullptr, 1,
        &pushConstantRange); // TODO add custom shader ubos

    CHECK_VK_RESULT_FATAL(g_device.createPipelineLayout(&pipelineLayoutInfo,
                                                        g_allocator,
                                                        &pipelineLayout),
                          "Failed to create pipeline layout.");

    vk::GraphicsPipelineCreateInfo pipelineInfo(
        vk::PipelineCreateFlags(), 2, shaderStages, &vertexInputInfo,
        &inputAssembly, nullptr, &viewportState, &rasterizer, &multisampling,
        nullptr, &colorBlending, &dynamicStateCreateInfo, pipelineLayout,
        g_renderPass, 0, nullptr, -1);

    CHECK_VK_RESULT_FATAL(
        g_device.createGraphicsPipelines(nullptr, 1, &pipelineInfo, g_allocator,
                                         &pipeline),
        "Failed to create pipeline layout.");

    g_device.destroyShaderModule(fragShaderModule, g_allocator);
    g_device.destroyShaderModule(vertShaderModule, g_allocator);
  }

public:
  const vk::PipelineLayout get_pipeline_layout() const {
    return pipelineLayout;
  }

  void bind_pipeline(const vk::CommandBuffer &commandbuffer) const {
    commandbuffer.bindPipeline(
        vk::PipelineBindPoint::eGraphics, // TODO Compute shader support
        pipeline);
  }

  void init(const std::string &vertPath, const std::string &fragPath) {
    init_pipeline(vertPath, fragPath);
  }

  void destroy() {
    g_device.destroyPipeline(pipeline, g_allocator);
    g_device.destroyPipelineLayout(pipelineLayout, g_allocator);
  }
};

Shader g_baseShader;

struct Texture {
private:
  Image image;

public:
  void init(const std::string &path) {
    int texWidth, texHeight, texChannels;
    stbi_uc *pixels =
        stbi_load(path.c_str(), &texWidth, &texHeight, &texChannels,
                  STBI_rgb_alpha); // TODO Support multiple pixel formats

    VkDeviceSize imageSize =
        texWidth * texHeight * 4; // TODO Support multiple pixel formats

    if (!pixels) {
      printf("[Error] Failed to load texture image %s", path.c_str());
      return;
    }

    Buffer stagingBuffer;
    stagingBuffer.allocate(imageSize, vk::BufferUsageFlagBits::eTransferSrc,
                           vk::MemoryPropertyFlagBits::eHostVisible |
                               vk::MemoryPropertyFlagBits::eHostCoherent);

    stagingBuffer.write(pixels, imageSize);
    stbi_image_free(pixels);

    image.allocate(texWidth, texHeight, vk::Format::eR8G8B8A8Unorm,
                   vk::ImageTiling::eOptimal,
                   vk::ImageUsageFlagBits::eTransferDst |
                       vk::ImageUsageFlagBits::eSampled,
                   vk::MemoryPropertyFlagBits::eDeviceLocal);
  }
};

struct UniformBufferObject {
private:
  std::vector<Buffer> buffers;
  std::vector<vk::DescriptorSet> descriptorSets;

  vk::DescriptorPool pool;

  size_t alignment = 0;

public:
  UniformBufferObject() = default;
  UniformBufferObject(UniformBufferObject &&other) {
    descriptorSets.resize(other.descriptorSets.size());
    for (size_t i = 0; i < other.descriptorSets.size(); ++i) {
      descriptorSets[i] = other.descriptorSets[i];
      other.descriptorSets[i] = nullptr;
    }

    pool = other.pool;
    other.pool = nullptr;

    buffers = std::move(other.buffers);
    alignment = other.alignment;
  }
  ~UniformBufferObject() { g_device.destroyDescriptorPool(pool, g_allocator); }

  size_t get_alignment() const { return alignment; }

  const vk::DescriptorSet &get_descriptor_set(size_t index) const {
    return descriptorSets[index];
  }

  template <typename T>
  void init(vk::DescriptorSetLayout layout,
            size_t nbElements = MAX_OBJECT_INSTANCES_PER_TEMPLATE) {
    if (T::DescriptorType == vk::DescriptorType::eUniformBufferDynamic) {
      size_t minUboAlignment = g_physicalDevice.getProperties()
                                   .limits.minUniformBufferOffsetAlignment;
      alignment = sizeof(T::Size);

      if (minUboAlignment > 0) {
        alignment = (alignment + minUboAlignment - 1) & ~(minUboAlignment - 1);
      }
    } else
      alignment = T::Size;

    size_t nbBuffers = g_swapchain.image_count();

    vk::DescriptorPoolSize poolSize{vk::DescriptorType::eUniformBuffer,
                                    static_cast<uint32_t>(nbBuffers)};

    vk::DescriptorPoolCreateInfo poolInfo{vk::DescriptorPoolCreateFlags(),
                                          static_cast<uint32_t>(nbBuffers), 1,
                                          &poolSize};

    CHECK_VK_RESULT_FATAL(
        g_device.createDescriptorPool(&poolInfo, g_allocator, &pool),
        "Failed to create descriptor pool.");

    buffers.resize(nbBuffers);
    for (size_t i = 0; i < buffers.size(); ++i)
      buffers[i].allocate(alignment * nbElements,
                          vk::BufferUsageFlagBits::eUniformBuffer,
                          vk::MemoryPropertyFlagBits::eHostCoherent |
                              vk::MemoryPropertyFlagBits::eHostVisible);

    std::vector<vk::DescriptorSetLayout> layouts(nbBuffers, layout);

    vk::DescriptorSetAllocateInfo allocInfo{
        pool, static_cast<uint32_t>(nbBuffers), layouts.data()};

    descriptorSets.resize(nbBuffers);
    CHECK_VK_RESULT_FATAL(
        g_device.allocateDescriptorSets(&allocInfo, descriptorSets.data()),
        "Failed to allocate descriptor sets.");

    for (size_t i = 0; i < descriptorSets.size(); ++i) {
      vk::DescriptorBufferInfo bufferInfo{buffers[i].get_handle(), 0, T::Size};

      vk::WriteDescriptorSet descriptorWrite{
          descriptorSets[i], 0,       0,           1,
          T::DescriptorType, nullptr, &bufferInfo, nullptr};

      g_device.updateDescriptorSets(1, &descriptorWrite, 0, nullptr);
    }
  }

  void write(uint32_t bufferIndex, const void *data, VkDeviceSize writeSize,
             uint64_t step) const {
    buffers[bufferIndex].write(data, writeSize, step * alignment);
  }
};

struct ModelInstanceInternal : ModelInstance {
private:
  Eigen::Matrix4f matrix;

  ModelInstanceInternal(uint64_t modelId, Vec3 _pos = Vec3::Zero(),
                        Vec3 _rot = Vec3::Zero(), Vec3 _scale = {1, 1, 1})
      : ModelInstance(modelId, _pos, _rot, _scale) {}

  void update_matrix() {
    if (upToDate)
      return;

    Eigen::Affine3f rotX, rotY, rotZ;
    Eigen::Affine3f translation;

    rotX = Eigen::AngleAxisf(rot.z, Eigen::Vector3f::UnitX());
    rotY = Eigen::AngleAxisf(rot.y, Eigen::Vector3f::UnitY());
    rotZ = Eigen::AngleAxisf(rot.x, Eigen::Vector3f::UnitZ());

    translation = Eigen::Translation3f(Eigen::Vector3f(pos.z, pos.x, pos.y));

    matrix = (translation * (rotX * rotZ * rotY) *
              Eigen::Scaling(scale.z, scale.x, scale.y))
                 .matrix();

    upToDate = true;
  }

  friend struct Model;
};

struct Model {
private:
  Buffer viBuffer;
  vk::DeviceSize vOffset = 0;

  uint64_t nbIndices = 0;

  // UniformBufferObject uboModelMat;

  std::vector<ModelInstanceInternal *> modelInstances;

  uint64_t id;

  template <typename T, size_t N, size_t O>
  vk::DeviceSize
  init_vi_buffer(const std::array<T, N> &vertexBuffer,
                 const std::array<VERTEX_INDICES_TYPE, O> &indexBuffer) {
    constexpr const VkDeviceSize iBufferSize = sizeof(VERTEX_INDICES_TYPE) * O;
    constexpr const VkDeviceSize vBufferSize = sizeof(T) * N;

    constexpr const VkDeviceSize bufferSize = iBufferSize + vBufferSize;
    constexpr const VkDeviceSize offset = iBufferSize;

    Buffer stagingBuffer;
    stagingBuffer.allocate(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                           vk::MemoryPropertyFlagBits::eHostVisible |
                               vk::MemoryPropertyFlagBits::eHostCoherent);

    stagingBuffer.write(indexBuffer.data(), iBufferSize);
    stagingBuffer.write(vertexBuffer.data(), vBufferSize, offset);

    viBuffer.allocate(bufferSize,
                      vk::BufferUsageFlagBits::eTransferDst |
                          vk::BufferUsageFlagBits::eVertexBuffer |
                          vk::BufferUsageFlagBits::eIndexBuffer,
                      vk::MemoryPropertyFlagBits::eDeviceLocal);

    stagingBuffer.copy_to(viBuffer, bufferSize, g_device,
                          g_graphicsQueue.handle);

    return offset;
  }

public:
  Model() {
    static uint64_t idCounter = 0;
    id = idCounter++;
  }
  Model(Model &&other) = default;
  ~Model() {
    for (size_t i = 0; i < modelInstances.size(); ++i)
      delete modelInstances[i];
  }

  uint64_t get_id() const { return id; }

  template <typename T, size_t N, size_t O>
  void init(const std::array<T, N> &vertices,
            const std::array<VERTEX_INDICES_TYPE, O> &indices) {
    vOffset = init_vi_buffer(vertices, indices);
    nbIndices = indices.size();

    // uboModelMat.init<UniformBufferInfo<UniformModelMat>>(
    //    g_baseDescriptorSetLayout);
  }

  template <typename Prim> void init_from_primitive() {
    init(Prim::Vertices, Prim::Indices);
  }

  void record(const vk::CommandBuffer &commandbuffer, Shader *shader,
              size_t index) const {
    commandbuffer.bindIndexBuffer(viBuffer.get_handle(), 0,
                                  VULKAN_INDICES_TYPE);
    commandbuffer.bindVertexBuffers(0, 1, &viBuffer.get_handle(), &vOffset);

    // uint64_t dynamicAlignment = uboModelMat.get_alignment();

    Eigen::Matrix4f view =
        Maths::LookAt(Eigen::Vector3f(8.f, 0.f, 2.f), Eigen::Vector3f::Zero(),
                      Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f proj = Maths::Perspective(
        90.f, g_extent.width / (float)g_extent.height, 0.1f, 10.f);
    proj(1, 1) *= -1;

    Eigen::Matrix4f vp = proj * view;
    commandbuffer.pushConstants(shader->get_pipeline_layout(),
                                vk::ShaderStageFlagBits::eVertex, 0,
                                sizeof(Eigen::Matrix4f), &vp);

    for (uint64_t i = 0; i < modelInstances.size(); ++i) {
      // uint32_t dynamicOffset = i * static_cast<uint32_t>(dynamicAlignment);

      // commandbuffer.bindDescriptorSets(
      //    vk::PipelineBindPoint::eGraphics, shader->get_pipeline_layout(), 0,
      //    1, &uboModelMat.get_descriptor_set(index), 1, &dynamicOffset);

      modelInstances[i]->update_matrix();

      commandbuffer.pushConstants(
          shader->get_pipeline_layout(), vk::ShaderStageFlagBits::eVertex,
          sizeof(Eigen::Matrix4f) * 2, sizeof(Eigen::Vector3f),
          &modelInstances[i]->color);

      commandbuffer.pushConstants(
          shader->get_pipeline_layout(), vk::ShaderStageFlagBits::eVertex,
          sizeof(Eigen::Matrix4f), sizeof(Eigen::Matrix4f),
          &modelInstances[i]->matrix);

      commandbuffer.drawIndexed(nbIndices, 1, 0, 0, 0);
    }
  }

  ModelInstance *spawn_instance(Vec3 pos = Vec3::Zero(),
                                Vec3 rot = Vec3::Zero(),
                                Vec3 scale = {1, 1, 1}) {
    ModelInstanceInternal *inst =
        new ModelInstanceInternal(uintptr_t(this), pos, rot, scale);
    modelInstances.push_back(inst);

    return modelInstances.back();
  }

  void destroy_instance(ModelInstance *inst) {
    for (size_t i = 0; i < modelInstances.size(); ++i) {
      if (modelInstances[i] == inst) {
        delete inst;
        modelInstances.erase(modelInstances.begin() + i);
      }
    }
  }

  // Eigen::Matrix4f update_mvp(uint32_t modelInstanceIndex) const {
  // UniformModelMat modelMat;

  // Eigen::Affine3f rotX, rotY, rotZ;
  // Eigen::Affine3f translation;

  // for (size_t i = 0; i < modelInstances.size(); ++i) {
  //  rotX =
  //      Eigen::AngleAxisf(modelInstances[i]->rot.z,
  //      Eigen::Vector3f::UnitX());
  //  rotY =
  //      Eigen::AngleAxisf(modelInstances[i]->rot.y,
  //      Eigen::Vector3f::UnitY());
  //  rotZ =
  //      Eigen::AngleAxisf(modelInstances[i]->rot.x,
  //      Eigen::Vector3f::UnitZ());

  //  translation = Eigen::Translation3f(
  //      Eigen::Vector3f(modelInstances[i]->pos.z, modelInstances[i]->pos.x,
  //                      modelInstances[i]->pos.y));

  // modelMat.model = (translation * (rotX * rotZ * rotY) *
  //                  Eigen::Scaling(modelInstances[i]->scale.z,
  //                                 modelInstances[i]->scale.x,
  //                                 modelInstances[i]->scale.y))
  //                     .matrix();
  // uboModelMat.write(currentImageIndex, &modelMat,
  //                  UniformBufferInfo<UniformModelMat>::Size, i);

  // modelInstances[i]->mat = (translation * (rotX * rotZ * rotY) *
  //                          Eigen::Scaling(modelInstances[i]->scale.z,
  //                                         modelInstances[i]->scale.x,
  //                                         modelInstances[i]->scale.y))
  //                             .matrix();
  //}
  //}
};

static struct {
  std::vector<vk::Framebuffer> framebuffers;
  std::vector<vk::CommandBuffer> commandbuffers;

  std::unordered_map<Shader *, std::vector<Model *>> models;

  void init_render_pass() {
    vk::AttachmentDescription colorAttachment(
        vk::AttachmentDescriptionFlags(), g_requiredFormat,
        vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eClear,
        vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
        vk::AttachmentStoreOp::eDontCare, vk::ImageLayout::eUndefined,
        vk::ImageLayout::ePresentSrcKHR);

    vk::AttachmentReference colorAttachmentRef(
        0, vk::ImageLayout::eColorAttachmentOptimal);

    vk::SubpassDescription subpass(vk::SubpassDescriptionFlags(),
                                   vk::PipelineBindPoint::eGraphics, 0, nullptr,
                                   1, &colorAttachmentRef);

    vk::SubpassDependency dependency(
        VK_SUBPASS_EXTERNAL, 0,
        vk::PipelineStageFlagBits::eColorAttachmentOutput,
        vk::PipelineStageFlagBits::eColorAttachmentOutput,
        (vk::AccessFlagBits)0,
        vk::AccessFlagBits::eColorAttachmentRead |
            vk::AccessFlagBits::eColorAttachmentWrite);

    vk::RenderPassCreateInfo renderPassInfo(vk::RenderPassCreateFlags(), 1,
                                            &colorAttachment, 1, &subpass, 1,
                                            &dependency);

    CHECK_VK_RESULT_FATAL(
        g_device.createRenderPass(&renderPassInfo, g_allocator, &g_renderPass),
        "Failed to create render pass.");
  }

  void destroy_render_pass() {
    g_device.destroyRenderPass(g_renderPass, g_allocator);
  }

  void init_shaders() {
    g_baseShader.init("../resources/shaders/spv/shader.vert.spv",
                      "../resources/shaders/spv/shader.frag.spv");
  }

  void destroy_shaders() { g_baseShader.destroy(); }

  void init_framebuffers() {
    framebuffers.resize(g_swapchain.image_count());

    for (size_t i = 0; i < framebuffers.size(); i++) {
      vk::ImageView attachments[] = {g_swapchain.get_image_view(i)};

      vk::FramebufferCreateInfo framebufferInfo(
          vk::FramebufferCreateFlags(), g_renderPass, 1, attachments,
          g_extent.width, g_extent.height, 1);

      CHECK_VK_RESULT_FATAL(g_device.createFramebuffer(&framebufferInfo,
                                                       g_allocator,
                                                       &framebuffers[i]),
                            "Failed to create framebuffer.");
    }
  }

  void destroy_framebuffers() {
    for (auto framebuffer : framebuffers)
      g_device.destroyFramebuffer(framebuffer, g_allocator);
  }

  void update_commandbuffer(size_t index) const {
    vk::CommandBufferBeginInfo beginInfo(
        vk::CommandBufferUsageFlagBits::eSimultaneousUse, nullptr);

    vk::Viewport viewport(0, 0, (float)g_extent.width, (float)g_extent.height,
                          0, 1);

    static constexpr const std::array<float, 4> clearColorArray = {
        {0.0f, 0.0f, 0.0f, 1.0f}};

    vk::ClearColorValue clearColorValue = clearColorArray;

    vk::ClearValue clearColor(clearColorValue);
    CHECK_VK_RESULT_FATAL(commandbuffers[index].begin(&beginInfo),
                          "Failed to begin recording command buffer.");

    vk::RenderPassBeginInfo renderPassInfo(g_renderPass, framebuffers[index],
                                           {{0, 0}, g_extent}, 1, &clearColor);

    commandbuffers[index].setViewport(
        0, 1,
        &viewport); // TODO buffers are recorded once so can't change viewport

    commandbuffers[index].beginRenderPass(&renderPassInfo,
                                          vk::SubpassContents::eInline);

    // TODO mt record commands
    for (const auto &pair : models) {
      pair.first->bind_pipeline(commandbuffers[index]);

      for (size_t j = 0; j < pair.second.size(); ++j) {
        pair.second[j]->record(commandbuffers[index], pair.first, index);
      }
    }

    commandbuffers[index].endRenderPass();

    commandbuffers[index]
        .end(); // Strangely, it does returns void instead of vk::Result so no
                // error checking is possible here
  }

  void init_commandbuffers() {
    commandbuffers.resize(framebuffers.size());

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

  void init() {
    g_swapchain.init();

    init_render_pass();
    init_shaders();

    init_framebuffers();

    init_commandbuffers();
  }

  void destroy() {
    destroy_framebuffers();
    destroy_commandbuffers();

    destroy_shaders();
    destroy_render_pass();

    g_swapchain.destroy();

    for (auto &pair : models)
      for (size_t j = 0; j < pair.second.size(); ++j)
        delete pair.second[j];

    models.clear();
  }

  void refresh() {
    int width = 0, height = 0;
    while (width == 0 || height == 0) {
      WindowHandler::GetFramebufferSize(&width, &height);
      WindowHandler::Update();
    }

    g_device.waitIdle();

    destroy();
    init();
  }

  // void update_transforms(uint32_t imageIndex) {
  //  for (const auto &pair : models) {
  //    for (size_t j = 0; j < pair.second.size(); ++j) {
  //      pair.second[j]->update_mvp(imageIndex);
  //    }
  //  }
  //}

} g_renderContext;

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
      {vk::ObjectType::eQueue, (uint64_t)((VkQueue)g_graphicsQueue.handle),
       "Graphics Queue"},
      g_dldy);
  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eQueue, (uint64_t)((VkQueue)g_presentQueue.handle),
       "Present Queue"},
      g_dldy);

  g_device.setDebugUtilsObjectNameEXT(
      {vk::ObjectType::eSwapchainKHR,
       (uint64_t)((VkSwapchainKHR)g_swapchain.get_handle()), "Swapchain"},
      g_dldy);

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

static std::array<vk::Semaphore, MAX_IN_FLIGHT_FRAMES>
    g_imageAvailableSemaphores;
static std::array<vk::Semaphore, MAX_IN_FLIGHT_FRAMES>
    g_renderFinishedSemaphores;

static std::array<vk::Fence, MAX_IN_FLIGHT_FRAMES> g_inFlightFences;

static size_t g_currentFrame = 0;

static void Draw() {
  g_device.waitForFences(1, &g_inFlightFences[g_currentFrame], VK_TRUE,
                         std::numeric_limits<uint64_t>::max());

  uint32_t imageIndex;
  vk::Result result = g_device.acquireNextImageKHR(
      g_swapchain.get_handle(), std::numeric_limits<uint64_t>::max(),
      g_imageAvailableSemaphores[g_currentFrame], nullptr, &imageIndex);

  if (result == vk::Result::eErrorOutOfDateKHR) {
    g_renderContext.refresh();
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

  // g_renderContext.update_transforms(imageIndex);
  g_renderContext.update_commandbuffer(imageIndex);

  vk::SubmitInfo submitInfo(1, &g_imageAvailableSemaphores[g_currentFrame],
                            &waitStage, 1,
                            &g_renderContext.commandbuffers[imageIndex], 1,
                            &g_renderFinishedSemaphores[g_currentFrame]);

  g_device.resetFences(1, &g_inFlightFences[g_currentFrame]);

  CHECK_VK_RESULT_FATAL(g_graphicsQueue.handle.submit(
                            1, &submitInfo, g_inFlightFences[g_currentFrame]),
                        "Failed to submit draw command buffer.");

  vk::PresentInfoKHR presentInfo(1, &g_renderFinishedSemaphores[g_currentFrame],
                                 1, &g_swapchain.get_handle(), &imageIndex,
                                 nullptr);

  result = g_presentQueue.handle.presentKHR(&presentInfo);
  if (result == vk::Result::eErrorOutOfDateKHR ||
      result == vk::Result::eSuboptimalKHR || g_framebufferResized) {
    g_framebufferResized = false;
    g_renderContext.refresh();
  } else
    CHECK_VK_RESULT_FATAL(result, "Failed to present Swapchain image.");

  g_currentFrame = (g_currentFrame + 1) % MAX_IN_FLIGHT_FRAMES;
}

//-----------------------------------------------------------------------------
// INITIALIZATION
//-----------------------------------------------------------------------------

static void CreateInstance() {
  vk::ApplicationInfo appInfo("Renderer", VK_MAKE_VERSION(1, 0, 0), "No Engine",
                              VK_MAKE_VERSION(1, 0, 0), VK_API_VERSION_1_1);

  vk::InstanceCreateInfo createInfo;
  createInfo.pApplicationInfo = &appInfo;

  const char **pExtensions = nullptr;

  uint32_t requiredExtensionsCount =
      WindowHandler::GetRequiredInstanceExtensions(pExtensions);
  std::vector<const char *> extensions(pExtensions,
                                       pExtensions + requiredExtensionsCount);

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
      vk::createInstance(&createInfo, g_allocator, &g_instance),
      "Failed to init vulkan instance");

  g_dldy.init(g_instance);

#ifndef NDEBUG
  for (size_t i = 0; i < g_debugMessengersInfos.size(); ++i)
    g_debugMessengers.push_back(g_instance.createDebugUtilsMessengerEXT(
        g_debugMessengersInfos[i].MakeCreateInfo(), g_allocator, g_dldy));
#endif
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
  const std::set<int> uniqueQueueFamilies = {indices.graphicsFamily,
                                             indices.presentFamily};

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

  CHECK_VK_RESULT_FATAL(
      g_physicalDevice.createDevice(&createInfo, g_allocator, &g_device),
      "Failed to create logical device");

  g_dldy.init(g_instance, g_device);

  g_graphicsQueue.index = 0;
  g_graphicsQueue.handle =
      g_device.getQueue(indices.graphicsFamily, g_graphicsQueue.index);

  g_presentQueue.index = 0;
  g_presentQueue.handle =
      g_device.getQueue(indices.presentFamily, g_presentQueue.index);
}

static void InitCommandPool() {
  QueueFamilyIndices queueFamilyIndices =
      GetQueueFamilies(g_physicalDevice, g_surface);

  vk::CommandPoolCreateInfo poolInfo(
      vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
      queueFamilyIndices.graphicsFamily);

  CHECK_VK_RESULT_FATAL(
      g_device.createCommandPool(&poolInfo, g_allocator, &g_commandPool),
      "Failed to create command pool.");
}

// void InitGlobalUBOs() {
//  for (size_t i = 0; i < g_globalDSLInfos.size(); ++i) {
//    vk::DescriptorSetLayoutCreateInfo layoutInfo(
//        vk::DescriptorSetLayoutCreateFlags(), 1, &g_globalDSLInfos[i]);
//
//    CHECK_VK_RESULT_FATAL(g_device.createDescriptorSetLayout(
//                              &layoutInfo, g_allocator, &g_globalDSL[i]),
//                          "Failed to create global descriptor set layout.");
//  }
//}

static void InitUsualDescriptorSetLayouts() {
  // UniformBufferInfo<UniformModelMat> info{.binding = 0};

  // vk::DescriptorSetLayoutBinding layoutBinding =
  //    info.make_descriptor_set_layout_binding();

  // vk::DescriptorSetLayoutCreateInfo layoutInfo(
  //    vk::DescriptorSetLayoutCreateFlags(), 1, &layoutBinding);

  // CHECK_VK_RESULT_FATAL(
  //    g_device.createDescriptorSetLayout(&layoutInfo, g_allocator,
  //                                       &g_baseDescriptorSetLayout),
  //    "Failed to create descriptor set layout.");
}

static void InitSyncBarriers() {
  vk::SemaphoreCreateInfo semaphoreInfo = {};

  vk::FenceCreateInfo fenceInfo = {vk::FenceCreateFlagBits::eSignaled};

  for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++) {
    CHECK_VK_RESULT_FATAL(
        g_device.createSemaphore(&semaphoreInfo, g_allocator,
                                 &g_imageAvailableSemaphores[i]),
        "Failed to create semaphores.");
    CHECK_VK_RESULT_FATAL(
        g_device.createSemaphore(&semaphoreInfo, g_allocator,
                                 &g_renderFinishedSemaphores[i]),
        "Failed to create semaphores.");

    CHECK_VK_RESULT_FATAL(
        g_device.createFence(&fenceInfo, g_allocator, &g_inFlightFences[i]),
        "Failed to create semaphores.");
  }
}

static void InitVulkan() {
  CreateInstance();
  CHECK_VK_RESULT_FATAL((vk::Result)WindowHandler::CreateSurface(
                            (VkInstance)g_instance,
                            (VkAllocationCallbacks *)g_allocator,
                            (VkSurfaceKHR *)&g_surface),
                        "Failed to create window surface");
  InitDevice();

  InitCommandPool();
  Allocator::Init(g_physicalDevice, g_device, g_graphicsQueue.index,
                g_allocator); // TODO Staging queue

  // InitGlobalUBOs();
  InitUsualDescriptorSetLayouts(); // TODO Does nothing for now

  g_renderContext.init();

  InitSyncBarriers();

#ifndef NDEBUG
  SetMainObjectsDebugNames();
#endif
}

//-----------------------------------------------------------------------------
// USER FUNCTIONS
//-----------------------------------------------------------------------------

void Init(unsigned int width, unsigned int height) {
  assert(width != 0 && height != 0);

  TRY_CATCH_BLOCK("Failed to init renderer",

                  WindowHandler::Init(width, height);
                  InitVulkan(););
}

bool Update() {
  TRY_CATCH_BLOCK("Failed to update renderer",

                  bool signal = WindowHandler::Update();
                  Draw();

                  return signal;);
}

void Shutdown() {
  g_device.waitIdle();

  TRY_CATCH_BLOCK(
      "Failed to shutdown renderer",
      for (size_t i = 0; i < MAX_IN_FLIGHT_FRAMES; i++) {
        g_device.destroySemaphore(g_imageAvailableSemaphores[i], g_allocator);
        g_device.destroySemaphore(g_renderFinishedSemaphores[i], g_allocator);

        g_device.destroyFence(g_inFlightFences[i], g_allocator);
      }

      g_renderContext.destroy();

      // g_device.destroyDescriptorSetLayout(g_baseDescriptorSetLayout,
      // g_allocator);

      // for (size_t i = 0; i < g_globalDSL.size(); ++i)
      //    g_device.destroyDescriptorSetLayout(g_globalDSL[i], g_allocator);

      Allocator::Destroy();

      g_device.destroyCommandPool(g_commandPool, g_allocator);

      g_device.destroy(g_allocator);

#ifndef NDEBUG
      for (size_t i = 0; i < g_debugMessengers.size(); ++i)
          g_instance.destroyDebugUtilsMessengerEXT(g_debugMessengers[i],
                                                   g_allocator, g_dldy);
#endif

      g_instance.destroySurfaceKHR(g_surface, g_allocator);
      g_instance.destroy(g_allocator);

      WindowHandler::Shutdown(););
}

void Run(unsigned int width, unsigned int height) {
  Init(width, height);

  while (Update())
    ;

  Shutdown();
}

void SetPresentMode(PresentMode presentMode) {
  g_swapchain.requiredPresentMode =
      static_cast<vk::PresentModeKHR>(presentMode);
  g_renderContext.refresh();
}

uint64_t CreateModel(EPrimitive primitive) {
  Model *model = new Model();
  switch (primitive) {
  case EPrimitive::Plane:
    model->init_from_primitive<Plane>();
    break;
  case EPrimitive::Cube:
    model->init_from_primitive<Cube>();
    break;
  }

  // TODO Set different shaders
  g_renderContext.models[&g_baseShader].push_back(model);

  return uintptr_t(g_renderContext.models[&g_baseShader].back());
}

ModelInstance *Spawn(uint64_t modelId, Vec3 pos, Vec3 rot, Vec3 scale) {
  return reinterpret_cast<Model *>(modelId)->spawn_instance(pos, rot, scale);
}

void Destroy(ModelInstance *instance) {
  reinterpret_cast<Model *>(instance->get_model_id())
      ->destroy_instance(instance);
}

} // namespace Renderer
