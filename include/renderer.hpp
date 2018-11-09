#pragma once

#include <stdint.h>

#ifdef VULKAN_RENDERER_DLL_EXPORTS
#define VULKAN_RENDERER_EXPORTS __declspec(dllexport)
#else
#define VULKAN_RENDERER_EXPORTS __declspec(dllimport)
#endif

namespace Renderer {
enum class PresentMode { Immediate = 0, Mailbox = 1, VSync = 2 };

VULKAN_RENDERER_EXPORTS void Init(unsigned int width, unsigned int height);

VULKAN_RENDERER_EXPORTS bool Update();
VULKAN_RENDERER_EXPORTS void Shutdown();

VULKAN_RENDERER_EXPORTS void Run(unsigned int width, unsigned int height);

VULKAN_RENDERER_EXPORTS void SetPresentMode(PresentMode presentMode);

struct Vec3 {
  float x, y, z = 0;

  static constexpr Vec3 Zero() { return {0, 0, 0}; }
};

struct ModelInstance {
protected:
  uint64_t modelID;
  bool upToDate = false;

  Vec3 pos;
  Vec3 rot;
  Vec3 scale;

  ModelInstance(Vec3 _pos = Vec3::Zero(), Vec3 _rot = Vec3::Zero(),
                Vec3 _scale = {1, 1, 1}) {
    pos = _pos;
    rot = _rot;
    scale = _scale;
  }

public:
  Vec3 color = {0.66f, 0.66f, 0.66f};

  void SetPosition(Vec3 position) {
    pos = position;
    upToDate = false;
  }

  void SetRotation(Vec3 rotation) {
    rot = rotation;
    upToDate = false;
  }

  void SetScale(Vec3 newScale) {
    scale = newScale;
    upToDate = false;
  }

  uint64_t get_model_id() const { return modelID; }
};

enum class EPrimitive { Plane, Cube };

VULKAN_RENDERER_EXPORTS uint64_t CreateModel(EPrimitive primitive);

VULKAN_RENDERER_EXPORTS ModelInstance *Spawn(uint64_t modelId,
                                             Vec3 pos = Vec3::Zero(),
                                             Vec3 rot = Vec3::Zero(),
                                             Vec3 scale = {1, 1, 1});

VULKAN_RENDERER_EXPORTS void Destroy(ModelInstance *instance);

} // namespace Renderer
