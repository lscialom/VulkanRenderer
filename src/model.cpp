#include "model.hpp"

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

Model::~Model() {
  for (size_t i = 0; i < modelInstances.size(); ++i)
    delete modelInstances[i];
}

void Model::init(const std::string &meshName) {

  mesh = ResourceManager::GetMesh(meshName);
  assert(mesh != nullptr); // TODO Manage mesh not loaded
}

// TODO index param only used for descriptors (that are unused for now).
void Model::record(const vk::CommandBuffer &commandbuffer, const Shader &shader,
                   size_t imageIndex) const {

  mesh->bind_buffer(commandbuffer, shader);

  // uint64_t dynamicAlignment = uboModelMat.get_alignment();

  ModelInstancePushConstant::ModelInstanceData miPc = {};

  for (uint64_t i = 0; i < modelInstances.size(); ++i) {
    // uint32_t dynamicOffset = i * static_cast<uint32_t>(dynamicAlignment);

    // commandbuffer.bindDescriptorSets(
    //    vk::PipelineBindPoint::eGraphics, shader->get_pipeline_layout(), 0,
    //    1, &uboModelMat.get_descriptor_set(imageIndex), 1, &dynamicOffset);

    modelInstances[i]->update_matrix();

    miPc.color.head<3>() << Maths::EigenizeVec3(modelInstances[i]->color);
    miPc.color.tail<1>() << modelInstances[i]->alpha;
    miPc.model = modelInstances[i]->transform.matrix();

    commandbuffer.pushConstants(shader.get_pipeline_layout(),
                                vk::ShaderStageFlagBits::eVertex |
                                    vk::ShaderStageFlagBits::eFragment,
                                0, sizeof(miPc), &miPc);

    mesh->draw(commandbuffer, shader);
  }
}

ModelInstance *Model::spawn_instance(Vec3 pos, Vec3 rot, Vec3 scale) {
  ModelInstanceInternal *inst =
      new ModelInstanceInternal(uintptr_t(this), pos, rot, scale);
  modelInstances.push_back(inst);

  return modelInstances.back();
}

void Model::destroy_instance(ModelInstance *inst) {
  for (size_t i = 0; i < modelInstances.size(); ++i) {
    if (modelInstances[i] == inst) {
      delete inst;
      modelInstances.erase(modelInstances.begin() + i);
    }
  }
}
} // namespace Renderer
