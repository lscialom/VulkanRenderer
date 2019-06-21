#include "nv_helpers_vk/BottomLevelASGenerator.h"
#include "nv_helpers_vk/TopLevelASGenerator.h"
#include "nv_helpers_vk/VKHelpers.h"

#include "memory.hpp"
#include "shader.hpp"

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

    Texture *diffuseMap = nullptr;
    Texture *alphaMap = nullptr;
  };

  using Queue = std::vector<Submesh>;

  GeometryInstance geometryInstance;
  Queue opaqueQueue;
  Queue transparentQueue;

  void drawQueue(const Queue &queue, const vk::CommandBuffer &commandbuffer,
                 const Shader &shader) const {

    for (size_t i = 0; i < queue.size(); ++i) {
      commandbuffer.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, shader.get_pipeline_layout(), 0, 1,
          queue[i].diffuseMap->get_descriptor_set(), 0, nullptr);

      commandbuffer.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, shader.get_pipeline_layout(), 1, 1,
          queue[i].alphaMap->get_descriptor_set(), 0, nullptr);

      commandbuffer.drawIndexed(queue[i].indexCount, 1, queue[i].indexOffset, 0,
                                0);
    }
  }

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
        std::string currentAlphaMap = currentShape.alphaMaps[j];

        uint64_t numFaces = 1;
        bool isTransparent = !currentAlphaMap.empty();

        while (++j < currentShape.diffuseMaps.size() &&
               currentShape.diffuseMaps[j] == currentDiffuseMap &&
               currentShape.alphaMaps[j] == currentAlphaMap) {

          ++numFaces;
        }

        Submesh submesh;

        submesh.indexCount = numFaces * 3;
        submesh.indexOffset = indexOffset;

        submesh.diffuseMap =
            currentDiffuseMap.empty()
                ? ResourceManager::GetTexture("default_texture")
                : ResourceManager::GetTexture(currentDiffuseMap);

        if (!isTransparent) {

          submesh.alphaMap = ResourceManager::GetTexture("default_texture");
          opaqueQueue.push_back(submesh);

        } else {

          submesh.alphaMap = ResourceManager::GetTexture(currentAlphaMap);
          transparentQueue.push_back(submesh);
        }

        indexOffset += submesh.indexCount;
      }

      ++shapeDataIterator;
    }
  }

  void bind_buffer(const vk::CommandBuffer &commandbuffer,
                   const Shader &shader) const {

    commandbuffer.bindIndexBuffer(geometryInstance.buffer.get_handle(), 0,
                                  VULKAN_INDICES_TYPE);
    commandbuffer.bindVertexBuffers(0, 1, &geometryInstance.buffer.get_handle(),
                                    &geometryInstance.vertexOffset);
  }

  // you must call bind_buffer before the first draw of the current update
  void draw(const vk::CommandBuffer &commandbuffer,
            const Shader &shader) const {

    drawQueue(opaqueQueue, commandbuffer, shader);
    drawQueue(transparentQueue, commandbuffer, shader);
  }
}; // namespace Renderer

} // namespace Renderer
