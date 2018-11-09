#include <renderer.hpp>

#include <chrono>

#define WIDTH 800
#define HEIGHT 600

#define M_PI 3.14159265359

int main() {
  Renderer::Init(WIDTH, HEIGHT);

  uint64_t cubeId = Renderer::CreateModel(Renderer::EPrimitive::Cube);
  Renderer::ModelInstance *inst0 = Renderer::Spawn(cubeId);

  Renderer::Vec3 baseRot = {M_PI / 4.f, 0, 0};
  Renderer::ModelInstance *inst1 =
      Renderer::Spawn(cubeId, {4.f, 0.f, 0.f}, baseRot);
  Renderer::ModelInstance *inst2 =
      Renderer::Spawn(cubeId, {-4.f, 0.f, 0.f}, baseRot);

  uint64_t planeId = Renderer::CreateModel(Renderer::EPrimitive::Square);
  Renderer::ModelInstance *plane = Renderer::Spawn(
      planeId, {0.f, -4.f, 0.f}, Renderer::Vec3::Zero(), {10.f, 10.f, 10.f});

  inst1->color = {1.f, 0.f, 0.f};
  inst2->color = {0.f, 0.f, 1.f};

  plane->color = {1, 1, 1};

  auto startTime = std::chrono::high_resolution_clock::now();

  while (Renderer::Update()) {
    auto currentTime = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float, std::chrono::seconds::period>(
                     currentTime - startTime)
                     .count();

    inst0->SetRotation({time * M_PI / 2.f, 0, 0});
    inst1->SetRotation({baseRot.x + time * M_PI / 2.f, 0, 0});
    inst2->SetRotation({-(baseRot.x + time * M_PI / 2.f), 0, 0});
  }

  Renderer::Shutdown();

  return 0;
}
