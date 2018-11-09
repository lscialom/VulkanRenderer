#pragma once

#ifdef VULKAN_RENDERER_DLL_EXPORTS
#define VULKAN_RENDERER_EXPORTS __declspec(dllexport)
#else
#define VULKAN_RENDERER_EXPORTS __declspec(dllimport)
#endif

namespace Renderer {
enum class PresentMode { Immediate = 0, Mailbox = 1, VSync = 2 };

struct Vec3 {
  float x, y, z = 0;

  static constexpr Vec3 Zero() { return {0, 0, 0}; }
};

struct ModelInstance {
  Vec3 pos;
  Vec3 rot;
  Vec3 scale;

  ModelInstance(Vec3 _pos = Vec3::Zero(), Vec3 _rot = Vec3::Zero(),
                Vec3 _scale = Vec3::Zero()) {
    pos = _pos;
    rot = _rot;
    scale = _scale;
  }
};

VULKAN_RENDERER_EXPORTS void Init(unsigned int width, unsigned int height);

VULKAN_RENDERER_EXPORTS bool Update();
VULKAN_RENDERER_EXPORTS void Shutdown();

VULKAN_RENDERER_EXPORTS void Run(unsigned int width, unsigned int height);

VULKAN_RENDERER_EXPORTS void SetPresentMode(PresentMode presentMode);
} // namespace Renderer
