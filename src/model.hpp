#pragma once

#include "memory.hpp"
#include "obj_loader.hpp"

namespace Renderer {
struct ModelInstanceInternal : ModelInstance {
  using Transform = Eigen::Transform<float, 3, Eigen::Affine>;

private:
  Transform transform;

  ModelInstanceInternal(uint64_t modelId, Vec3 _pos = Vec3::Zero(),
                        Vec3 _rot = Vec3::Zero(), Vec3 _scale = Vec3::One())
      : ModelInstance(modelId, _pos, _rot, _scale) {}

  void update_matrix() {
    if (upToDate)
      return;

    Eigen::Vector3f position = Maths::EigenizeVec3(pos);
    Eigen::Vector3f rotation = Maths::EigenizeVec3(rot);

    Eigen::AngleAxisf rotX, rotY, rotZ;

    rotX = Eigen::AngleAxisf(rotation.x(), Eigen::Vector3f::UnitX());
    rotY = Eigen::AngleAxisf(rotation.y(), Eigen::Vector3f::UnitY());
    rotZ = Eigen::AngleAxisf(rotation.z(), Eigen::Vector3f::UnitZ());

    transform.fromPositionOrientationScale(position, rotZ * rotY * rotX,
                                           Maths::EigenizeVec3(scale));

    upToDate = true;
  }

  friend struct Model;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // To ensure Eigen type members are aligned
                                  // for proper vectorization
};

struct Model {
private:
  Buffer viBuffer;
  vk::DeviceSize vOffset = 0;

  uint64_t nbIndices = 0;

  // UniformBufferObject uboModelMat;

  std::vector<ModelInstanceInternal *> modelInstances;

  template <typename T, size_t N, size_t O>
  vk::DeviceSize
  init_vi_buffer(const std::array<T, N> &vertexBuffer,
                 const std::array<VERTEX_INDICES_TYPE, O> &indexBuffer) {
    constexpr const VkDeviceSize iBufferSize = sizeof(VERTEX_INDICES_TYPE) * O;
    constexpr const VkDeviceSize vBufferSize = sizeof(T) * N;

    constexpr const VkDeviceSize bufferSize = iBufferSize + vBufferSize;
    constexpr const VkDeviceSize offset = iBufferSize;

    // Making the staging by ourselves is more optimized than the auto one in
    // Buffer::write in this case (because ATM we would need 2 calls)

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

    stagingBuffer.copy_to(viBuffer, bufferSize);

    return offset;
  }

  template <typename T>
  vk::DeviceSize
  init_vi_buffer(const std::vector<T> &vertexBuffer,
                 const std::vector<VERTEX_INDICES_TYPE> &indexBuffer) {
    const VkDeviceSize iBufferSize =
        sizeof(VERTEX_INDICES_TYPE) * indexBuffer.size();
    const VkDeviceSize vBufferSize = sizeof(T) * vertexBuffer.size();

    const VkDeviceSize bufferSize = iBufferSize + vBufferSize;
    const VkDeviceSize offset = iBufferSize;

    // Making the staging by ourselves is more optimized than the auto one in
    // Buffer::write in this case (because ATM we would need 2 calls)

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

    stagingBuffer.copy_to(viBuffer, bufferSize);

    return offset;
  }

public:
  Model() = default;
  Model(Model &&other) = default;
  ~Model() {
    for (size_t i = 0; i < modelInstances.size(); ++i)
      delete modelInstances[i];
  }

  template <typename T, size_t N, size_t O>
  void init(const std::array<T, N> &vertices,
            const std::array<VERTEX_INDICES_TYPE, O> &indices) {
    vOffset = init_vi_buffer(vertices, indices);
    nbIndices = indices.size();

    // uboModelMat.init<UniformBufferInfo<UniformModelMat>>(
    //    g_baseDescriptorSetLayout);
  }

  template <typename T>
  void init(const std::vector<T> &vertices,
            const std::vector<VERTEX_INDICES_TYPE> &indices) {
    vOffset = init_vi_buffer(vertices, indices);
    nbIndices = indices.size();
  }

  template <typename Prim> void init_from_primitive() {
    init(Prim::Vertices, Prim::Indices);
  }

  void init_from_obj_file(const std::string &objFilename) {
    std::vector<LiteralVertex> vertices;
    std::vector<VERTEX_INDICES_TYPE> indices;
    ObjLoader::LoadObj(objFilename, vertices, indices);
    init(vertices, indices);
  }

  // TODO index param only used for descriptors (that are unused for now).
  void record(const vk::CommandBuffer &commandbuffer, const Shader &shader,
              size_t imageIndex) const {
    commandbuffer.bindIndexBuffer(viBuffer.get_handle(), 0,
                                  VULKAN_INDICES_TYPE);
    commandbuffer.bindVertexBuffers(0, 1, &viBuffer.get_handle(), &vOffset);

    // uint64_t dynamicAlignment = uboModelMat.get_alignment();

    ModelInstancePushConstant miPc = {};

    // commandbuffer.pushConstants(shader.get_pipeline_layout(),
    //                            vk::ShaderStageFlagBits::eVertex, 0,
    //                            sizeof(Eigen::Matrix4f), &Camera::Matrix);

    for (uint64_t i = 0; i < modelInstances.size(); ++i) {
      // uint32_t dynamicOffset = i * static_cast<uint32_t>(dynamicAlignment);

      // commandbuffer.bindDescriptorSets(
      //    vk::PipelineBindPoint::eGraphics, shader->get_pipeline_layout(), 0,
      //    1, &uboModelMat.get_descriptor_set(imageIndex), 1, &dynamicOffset);

      modelInstances[i]->update_matrix();

      miPc.color.head<3>() << Maths::EigenizeVec3(modelInstances[i]->color);
      miPc.color.tail<1>() << modelInstances[i]->shininess;
      miPc.model = modelInstances[i]->transform.matrix();

      // commandbuffer.pushConstants(
      //    shader.get_pipeline_layout(), vk::ShaderStageFlagBits::eVertex,
      //    sizeof(Eigen::Matrix4f) * 2, sizeof(Eigen::Vector3f),
      //    &modelInstances[i]->color);

      // commandbuffer.pushConstants(
      //    shader.get_pipeline_layout(), vk::ShaderStageFlagBits::eVertex,
      //    sizeof(Eigen::Matrix4f), sizeof(Eigen::Matrix4f),
      //    &modelInstances[i]->transform.matrix());

      commandbuffer.pushConstants(shader.get_pipeline_layout(),
                                  vk::ShaderStageFlagBits::eVertex |
                                      vk::ShaderStageFlagBits::eFragment,
                                  0, sizeof(miPc), &miPc);

      commandbuffer.drawIndexed(nbIndices, 1, 0, 0, 0);
    }
  }

  ModelInstance *spawn_instance(Vec3 pos = Vec3::Zero(),
                                Vec3 rot = Vec3::Zero(),
                                Vec3 scale = Vec3::One()) {
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
} // namespace Renderer
