#pragma once

#include "renderer.hpp"

#include "memory.hpp"

// TODO Remove when got rid of Primitive struct
#include "obj_loader.hpp"

#include "resource_manager.hpp"

#include "mesh.hpp"

namespace Renderer {

struct ModelInstanceInternal;

struct Model {
private:
  // UniformBufferObject uboModelMat;

  Mesh *mesh;
  std::vector<ModelInstanceInternal *> modelInstances;

public:
  Model() = default;
  Model(Model &&other) = default;
  ~Model();

  template <typename Prim> void init_from_primitive() {

    const std::array<ObjLoader::ShapeData, 1> shapeData = {
        {{Prim::Indices.size()}}};

    mesh->init(Prim::Vertices.begin(), Prim::Vertices.end(),
               Prim::Indices.begin(), Prim::Indices.end(), shapeData.begin(),
               shapeData.end());
  }

  void init(const std::string &meshName);

  // TODO index param only used for descriptors (that are unused for now).
  void record(const vk::CommandBuffer &commandbuffer, const Shader &shader,
              size_t imageIndex) const;

  ModelInstance *spawn_instance(Vec3 pos = Vec3::Zero(),
                                Vec3 rot = Vec3::Zero(),
                                Vec3 scale = Vec3::One());

  void destroy_instance(ModelInstance *inst);
};
} // namespace Renderer
