#pragma once

#include <stdint.h>
#include <string>

#ifdef VULKAN_RENDERER_DLL_EXPORTS
#define VULKAN_RENDERER_EXPORTS __declspec(dllexport)
#else
#define VULKAN_RENDERER_EXPORTS __declspec(dllimport)
#endif

namespace Renderer {
enum class PresentMode { Immediate = 0, Mailbox = 1, VSync = 2 };

namespace Config {
VULKAN_RENDERER_EXPORTS extern bool ShowGBuffer;
VULKAN_RENDERER_EXPORTS extern bool SSAOEnable;
} // namespace Config

VULKAN_RENDERER_EXPORTS void Init(unsigned int width, unsigned int height,
                                  void *windowHandle);

VULKAN_RENDERER_EXPORTS void Update();
VULKAN_RENDERER_EXPORTS void Shutdown();

VULKAN_RENDERER_EXPORTS void Resize(unsigned int width, unsigned int height);

VULKAN_RENDERER_EXPORTS void SetPresentMode(PresentMode presentMode);
VULKAN_RENDERER_EXPORTS void GetCurrentResolution(int &w, int &h);

struct Vec3 {
  float x, y, z = 0;

  const Vec3 operator+(const Vec3 &other) {
    return {x + other.x, y + other.y, z + other.z};
  }

  const Vec3 operator*(float a) { return {x * a, y * a, z * a}; }

  static constexpr Vec3 Zero() { return {0, 0, 0}; }
  static constexpr Vec3 One() { return {1, 1, 1}; }

  static constexpr Vec3 UnitX() { return {1, 0, 0}; }
  static constexpr Vec3 UnitY() { return {0, 1, 0}; }
  static constexpr Vec3 UnitZ() { return {0, 0, 1}; }
};

namespace Color {
static constexpr Vec3 Black = Vec3::Zero();
static constexpr Vec3 Grey = {0.66f, 0.66f, 0.66f};
static constexpr Vec3 White = Vec3::One();

static constexpr Vec3 Red = Vec3::UnitX();
static constexpr Vec3 Green = Vec3::UnitY();
static constexpr Vec3 Blue = Vec3::UnitZ();
} // namespace Color

namespace Camera {
VULKAN_RENDERER_EXPORTS extern float Fov;
VULKAN_RENDERER_EXPORTS extern float Near;
VULKAN_RENDERER_EXPORTS extern float Far;

VULKAN_RENDERER_EXPORTS extern Vec3 Position;
VULKAN_RENDERER_EXPORTS extern Vec3 Rotation;

} // namespace Camera

enum class LightType { Directional = 0, Point = 1 };

struct Light {
  // if LightType is Directional, vector is a direction, else if LightType is a
  // Point, vector is a position
  Vec3 vector;

  Vec3 color;

  float ambientFactor = 0.1f;

  float maxDist = 65.f;

  LightType lightType = LightType::Directional;

private:
  Light() = default;
  Light(Vec3 vec = Vec3::Zero(), Vec3 col = Color::White,
        float ambientStrength = 0.1f)
      : vector{vec}, color{col}, ambientFactor{ambientStrength} {};

  friend struct RenderContext;
};

struct ModelInstance {
protected:
  uint64_t modelID;
  bool upToDate = false;

  Vec3 pos;
  Vec3 rot;
  Vec3 scale;

  ModelInstance(uint64_t modelTemplateID, Vec3 _pos = Vec3::Zero(),
                Vec3 _rot = Vec3::Zero(), Vec3 _scale = Vec3::One()) {
    modelID = modelTemplateID;

    pos = _pos;
    rot = _rot;
    scale = _scale;
  }

public:
  Vec3 color = Color::White;
  float shininess = 0.5f;

  Vec3 GetPosition() const { return pos; }
  Vec3 GetRotation() const { return rot; }
  Vec3 GetScale() const { return scale; }

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

  void SetPosition(float x = 0, float y = 0, float z = 0) {
    SetPosition({x, y, z});
  }

  void SetRotation(float x = 0, float y = 0, float z = 0) {
    SetRotation({x, y, z});
  }

  void SetScale(float x = 0, float y = 0, float z = 0) { SetScale({x, y, z}); }

  uint64_t get_model_id() const { return modelID; }
};

enum class EPrimitive { Plane, Cube };

VULKAN_RENDERER_EXPORTS void LoadTexture(const std::string &texturePath,
                                         const std::string &name);

VULKAN_RENDERER_EXPORTS uint64_t CreateModelFromPrimitive(EPrimitive primitive);
VULKAN_RENDERER_EXPORTS uint64_t
CreateModelFromObj(const std::string &objFilename,
                   const std::string &textureName = "default_texture");

VULKAN_RENDERER_EXPORTS ModelInstance *Spawn(uint64_t modelId,
                                             Vec3 pos = Vec3::Zero(),
                                             Vec3 rot = Vec3::Zero(),
                                             Vec3 scale = Vec3::One());

VULKAN_RENDERER_EXPORTS void Destroy(ModelInstance *instance);

VULKAN_RENDERER_EXPORTS Light *SpawnLight(Vec3 position = Vec3::Zero(),
                                          Vec3 color = Color::White,
                                          float ambientStrength = 0.1f);

} // namespace Renderer
