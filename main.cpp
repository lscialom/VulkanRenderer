#include <window_handler.hpp>

#include <renderer.hpp>

#include <chrono>

#define WIDTH 800
#define HEIGHT 600

#define M_PI 3.14159265359

bool resizeCallback = false;
int main() {
  WindowHandler::Init(WIDTH, HEIGHT);
  WindowHandler::SetFramebufferSizeCallbackSignal(&resizeCallback);

  Renderer::Init(WIDTH, HEIGHT, WindowHandler::GetHandle());

  Renderer::SetFov(90.f);
  Renderer::SetFar(100.f);

  uint64_t cubeId = Renderer::CreateModel(Renderer::EPrimitive::Cube);
  Renderer::ModelInstance *inst0 = Renderer::Spawn(cubeId);

  Renderer::Vec3 baseRot = {M_PI / 4.f, 0, 0};
  Renderer::ModelInstance *inst1 =
      Renderer::Spawn(cubeId, {4.f, 0.f, 0.f}, baseRot);
  Renderer::ModelInstance *inst2 =
      Renderer::Spawn(cubeId, {-4.f, 0.f, 0.f}, baseRot);
  Renderer::ModelInstance *inst3 = Renderer::Spawn(cubeId, {0.f, 4.f, 0.f});

  uint64_t planeId = Renderer::CreateModel(Renderer::EPrimitive::Plane);
  Renderer::ModelInstance *plane = Renderer::Spawn(
      planeId, {0.f, -4.f, 0.f}, Renderer::Vec3::Zero(), {10.f, 10.f, 10.f});

  inst1->color = Renderer::Color::Red;
  inst2->color = Renderer::Color::Blue;
  inst3->color = Renderer::Color::Green;

  plane->color = Renderer::Color::White;

  auto startTime = std::chrono::high_resolution_clock::now();

  while (WindowHandler::Update()) {
    // TODO Move into a function
    if (resizeCallback) {
      int width = 0, height = 0;
      while (width == 0 || height == 0) {
        WindowHandler::GetFramebufferSize(&width, &height);
        WindowHandler::Update();
      }

      resizeCallback = false;
      Renderer::Resize(width, height);
    }

    Renderer::Update();

    auto currentTime = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float, std::chrono::seconds::period>(
                     currentTime - startTime)
                     .count();

    inst0->SetRotation({time * M_PI / 2.f, 0, 0});
    inst1->SetRotation({baseRot.x + time * M_PI / 2.f, 0, 0});
    inst2->SetRotation({-(baseRot.x + time * M_PI / 2.f), 0, 0});
    inst3->SetRotation({time * M_PI / 2.f, 0, 0});

    inst3->SetScale({0.5f, 0.5f, 1.f});
  }

  Renderer::Shutdown();
  WindowHandler::Shutdown();

  return 0;
}
