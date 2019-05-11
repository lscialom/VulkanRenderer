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
  struct FacePool {
    uint64_t numFaces = 0;
    Texture *texture;
  };

  struct SubMesh {
    VERTEX_INDICES_TYPE indexCount;
    std::vector<FacePool> facePools;
  };

  GeometryInstance geometryInstance;
  std::vector<SubMesh> submeshes;

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

    submeshes.resize(shapeDataEnd - shapeDataBegin);

    auto shapeDataIterator = shapeDataBegin;
    for (size_t i = 0; i < submeshes.size(); ++i) {
      auto currentShape = *shapeDataIterator;

      submeshes[i].indexCount = currentShape.indexCount;

      for (size_t j = 0; j < currentShape.textures.size();) {
        std::string currentTex = currentShape.textures[j];

        uint64_t numFaces = 1;
        while (++j < currentShape.textures.size() &&
               currentShape.textures[j] == currentTex) {

          ++numFaces;
        }

        submeshes[i].facePools.push_back(
            {numFaces, currentTex.empty()
                           ? ResourceManager::GetTexture("default_texture")
                           : ResourceManager::GetTexture(currentTex)});
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

    VERTEX_INDICES_TYPE currentIndex = 0;

    for (size_t i = 0; i < submeshes.size(); ++i) {

      for (size_t j = 0; j < submeshes[i].facePools.size(); ++j) {

        commandbuffer.bindDescriptorSets(
            vk::PipelineBindPoint::eGraphics, shader.get_pipeline_layout(), 0,
            1, submeshes[i].facePools[j].texture->get_descriptor_set(), 0,
            nullptr);

        commandbuffer.drawIndexed(submeshes[i].facePools[j].numFaces * 3, 1,
                                  currentIndex, 0, 0);

        currentIndex += submeshes[i].facePools[j].numFaces * 3;
      }
    }
  }
}; // namespace Renderer

} // namespace Renderer
