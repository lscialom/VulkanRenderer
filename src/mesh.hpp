#pragma once

#include "nv_helpers_vk/BottomLevelASGenerator.h"
#include "nv_helpers_vk/TopLevelASGenerator.h"
#include "nv_helpers_vk/VKHelpers.h"

#include "memory.hpp"
#include "shader.hpp"

#include "common_resources.hpp"

#include "texture.hpp"

namespace Renderer {

struct GeometryInstance {
  Buffer buffer;

  uint32_t vertexCount;
  vk::DeviceSize vertexOffset;

  uint32_t indexCount;
  vk::DeviceSize indexOffset;
  // Eigen::Matrix4f transform;
};

struct AccelerationStructure {
  Buffer scratchBuffer;
  Buffer resultBuffer;
  Buffer instancesBuffer;

  vk::AccelerationStructureNV structure;
};

struct Mesh {
private:
  struct Submesh {
    uint64_t indexCount = 0;
    VERTEX_INDICES_TYPE indexOffset = 0;

    Eigen::Vector3f diffuseColor;

    vk::Sampler diffuseSampler = nullptr;
    vk::Sampler normalSampler = nullptr;

    DescriptorSet descriptorSet;

    Submesh() = default;
    Submesh(const Submesh &other) = delete;

    Submesh(Submesh &&other) { *this = std::move(other); }

    Submesh &operator=(const Submesh &other) = delete;

    Submesh &operator=(Submesh &&other);

    void init_descriptor(const std::string &diffuse, const std::string &normal);

    ~Submesh();
  };

  using Queue = std::vector<Submesh>;

  GeometryInstance geometryInstance;
  Queue opaqueQueue;
  Queue transparentQueue;

  void draw_queue(const Queue &queue, const vk::CommandBuffer &commandbuffer,
                  const Shader &shader) const;

public:
  // /!\ Data must be contiguous
  template <typename Iter1, typename Iter2, typename Iter3>
  void init(Iter1 vertexBegin, Iter1 vertexEnd, Iter2 indexBegin,
            Iter2 indexEnd, Iter3 shapeDataBegin, Iter3 shapeDataEnd) {

    using VertexType = std::decay_t<decltype(*vertexBegin)>;
    using IndexType = std::decay_t<decltype(*indexBegin)>;

    const uint32_t indexCount = indexEnd - indexBegin;
    const uint32_t vertexCount = vertexEnd - vertexBegin;

    const VkDeviceSize iBufferSize = sizeof(IndexType) * indexCount;
    const VkDeviceSize vBufferSize = sizeof(VertexType) * vertexCount;

    const VkDeviceSize bufferSize = iBufferSize + vBufferSize;
    const VkDeviceSize vertexOffset = iBufferSize;

    // Making the staging by ourselves is more optimized than the auto one in
    // Buffer::write in this case (because ATM we would need 2 calls)

    Buffer stagingBuffer;
    stagingBuffer.allocate(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
                           vk::MemoryPropertyFlagBits::eHostVisible |
                               vk::MemoryPropertyFlagBits::eHostCoherent);

    // pointers to first element
    stagingBuffer.write(&*indexBegin, iBufferSize);
    stagingBuffer.write(&*vertexBegin, vBufferSize, vertexOffset);

    geometryInstance.buffer.allocate(
        bufferSize,
        vk::BufferUsageFlagBits::eTransferDst |
            vk::BufferUsageFlagBits::eVertexBuffer |
            vk::BufferUsageFlagBits::eIndexBuffer,
        vk::MemoryPropertyFlagBits::eDeviceLocal);

    stagingBuffer.copy_to(geometryInstance.buffer, bufferSize);

    geometryInstance.indexOffset = 0;
    geometryInstance.indexCount = indexCount;

    geometryInstance.vertexCount = vertexCount;
    geometryInstance.vertexOffset = vertexOffset;

    size_t numShapes = shapeDataEnd - shapeDataBegin;

    auto shapeDataIterator = shapeDataBegin;
    for (size_t i = 0; i < numShapes; ++i) {
      auto currentShape = *shapeDataIterator;

      VERTEX_INDICES_TYPE indexOffset = currentShape.indexOffset;

      for (size_t j = 0; j < currentShape.diffuseMaps.size();) {
        std::string currentDiffuseMap = currentShape.diffuseMaps[j];
        std::string currentNormalMap = currentShape.normalMaps[j];
        float currentDissolve = currentShape.dissolves[j];

        Eigen::Vector3f currentDiffuseColor = currentShape.diffuseColors[j];

        uint64_t numFaces = 1;
        bool isTransparent = (currentDissolve != 0);

        while (++j < currentShape.diffuseMaps.size() &&
               currentShape.diffuseMaps[j] == currentDiffuseMap &&
               currentShape.normalMaps[j] == currentNormalMap &&
               currentShape.dissolves[j] == currentDissolve &&
               currentShape.diffuseColors[j] == currentDiffuseColor) {

          ++numFaces;
        }

        Submesh submesh;
        submesh.indexCount = numFaces * 3;
        submesh.indexOffset = indexOffset;

        submesh.diffuseColor = currentDiffuseColor;

        submesh.init_descriptor(currentDiffuseMap, currentNormalMap);

        if (!isTransparent)
          opaqueQueue.emplace_back(std::move(submesh));
        else
          transparentQueue.emplace_back(std::move(submesh));

        indexOffset += submesh.indexCount;
      }

      ++shapeDataIterator;
    }
  }

  void bind_buffer(const vk::CommandBuffer &commandbuffer,
                   const Shader &shader) const;

  // you must call bind_buffer before the first draw of the current update
  void draw(const vk::CommandBuffer &commandbuffer, const Shader &shader) const;
}; // namespace Renderer

} // namespace Renderer
