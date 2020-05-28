#include <window_handler.hpp>

#include <renderer.hpp>

#include <ext_imgui.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <string>

#define WIDTH 1920
#define HEIGHT 1080

#define M_PI 3.14159265359f

static float frameTime = 0;

static bool resizeCallback = false;

#define RELEASE 0
#define PRESS 1
#define REPEAT 2

static bool W = false;
static bool A = false;
static bool S = false;
static bool D = false;

static bool esc = false;

static bool mouseCaptured = true;

static void KeyCallback(int keycode, int scancode, int action, int mods) {
#define KEY_BOOL(key)                                                          \
  if (keycode == (#key)[0]) {                                                  \
    if (action == RELEASE)                                                     \
      key = false;                                                             \
    else                                                                       \
      key = true;                                                              \
  }

  KEY_BOOL(W)
  KEY_BOOL(A)
  KEY_BOOL(S)
  KEY_BOOL(D)

  if (keycode == 'C' && action == PRESS) {
    if (mouseCaptured) {
      WindowHandler::ReleaseMouse();
      mouseCaptured = false;
    } else {
      WindowHandler::CaptureMouse();
      mouseCaptured = true;
    }
  }

  if (keycode == 256 && action == PRESS)
    esc = true;

#undef KEY_BOOL
}

static float xMouse = 0;
static float yMouse = 0;
static void CursorPosCallback(double x, double y) {
  if (mouseCaptured) {

    float dx = (x - xMouse);
    float dy = (y - yMouse);

    float dTime = frameTime * 0.5f * 0.125f;

    Renderer::Camera::Rotation =
        Renderer::Camera::Rotation +
        Renderer::Vec3{M_PI * dy * dTime, M_PI * -dx * dTime, 0};

    static constexpr float MAX_ROT_X = M_PI / 3.f;

    if (Renderer::Camera::Rotation.x > MAX_ROT_X)
      Renderer::Camera::Rotation.x = MAX_ROT_X;
    else if (Renderer::Camera::Rotation.x < -MAX_ROT_X)
      Renderer::Camera::Rotation.x = -MAX_ROT_X;
  }

  xMouse = x;
  yMouse = y;
}

#undef RELEASE
#undef PRESS
#undef REPEAT

#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3

static Renderer::Vec3 ForwardVector(Renderer::Vec3 euler, int dir) {
  Eigen::AngleAxisf rotX, rotY, rotZ;

  rotX = Eigen::AngleAxisf(euler.x, Eigen::Vector3f::UnitX());
  rotY = Eigen::AngleAxisf(euler.y, Eigen::Vector3f::UnitY());
  rotZ = Eigen::AngleAxisf(euler.z, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf rot = rotZ * rotY * rotX;

  Eigen::Vector3f direction;
  if (dir == FORWARD)
    direction = Eigen::Vector3f::UnitZ();
  else if (dir == BACKWARD)
    direction = -Eigen::Vector3f::UnitZ();
  else if (dir == LEFT)
    direction = Eigen::Vector3f::UnitX();
  else if (dir == RIGHT)
    direction = -Eigen::Vector3f::UnitX();

  Eigen::Vector3f res = (rot * direction).normalized();

  return {res.x(), res.y(), res.z()};
}

static void UpdateInputs() {
  float delta = frameTime * 10;

  if (W)
    Renderer::Camera::Position =
        Renderer::Camera::Position +
        ForwardVector(Renderer::Camera::Rotation, FORWARD) * delta;
  if (A)
    Renderer::Camera::Position =
        Renderer::Camera::Position +
        ForwardVector(Renderer::Camera::Rotation, LEFT) * delta;
  if (S)
    Renderer::Camera::Position =
        Renderer::Camera::Position +
        ForwardVector(Renderer::Camera::Rotation, BACKWARD) * delta;
  if (D)
    Renderer::Camera::Position =
        Renderer::Camera::Position +
        ForwardVector(Renderer::Camera::Rotation, RIGHT) * delta;
}

#undef FORWARD
#undef BACKWARD
#undef LEFT
#undef RIGHT

static ExtImGui::Console *console = nullptr;
static ExtImGui::Performances *perf = nullptr;

static void SetPresentModeConsole(const std::vector<std::string> &args) {
  if (args.empty())
    console->AddLog("Invalid number of arguments. One is required.");

  std::string imm = "immediate";
  std::string mb = "mailbox";
  std::string vs = "vsync";

  std::string arg = args[0];

  if (arg == imm)
    Renderer::SetPresentMode(Renderer::PresentMode::Immediate);
  else if (arg == mb)
    Renderer::SetPresentMode(Renderer::PresentMode::Mailbox);
  else if (arg == vs)
    Renderer::SetPresentMode(Renderer::PresentMode::VSync);
  else
    console->AddLog("Invalid argument. Must be either \"immediate\", "
                    "\"mailbox\" or \"vsync\"");
}

static void ShowFPSConsole(const std::vector<std::string> &args) {
  perf->open = true;
}

static void ToggleGBuffer(const std::vector<std::string> &args) {
  Renderer::Config::ShowGBuffer = !Renderer::Config::ShowGBuffer;
}

static void ToggleSSAO(const std::vector<std::string> &args) {
  Renderer::Config::SSAOEnable = !Renderer::Config::SSAOEnable;
}

static void GetCameraPosition(const std::vector<std::string> &args) {
  float x = Renderer::Camera::Position.x;
  float y = Renderer::Camera::Position.y;
  float z = Renderer::Camera::Position.z;

  console->AddLog("%f; %f; %f", x, y, z);
}

static void GetCameraRotation(const std::vector<std::string> &args) {
  float x = Renderer::Camera::Rotation.x;
  float y = Renderer::Camera::Rotation.y;
  float z = Renderer::Camera::Rotation.z;

  console->AddLog("%f; %f; %f", x, y, z);
}

int main() {
  WindowHandler::Init(WIDTH, HEIGHT);
  WindowHandler::SetFramebufferSizeCallbackSignal(&resizeCallback);

  WindowHandler::AddKeyCallback(KeyCallback);
  WindowHandler::AddCursorPosCallback(CursorPosCallback);

  Renderer::Init(WIDTH, HEIGHT, WindowHandler::GetHandle());

  ExtImGui::Init(600, 400);
  console = ExtImGui::CreateConsole();
  perf = ExtImGui::CreatePerformancesWidget();

  console->AddCommand("spm", SetPresentModeConsole);
  console->AddCommand("showfps", ShowFPSConsole);
  console->AddCommand("tgb", ToggleGBuffer);
  console->AddCommand("tssao", ToggleSSAO);
  console->AddCommand("gcp", GetCameraPosition);
  console->AddCommand("gcr", GetCameraRotation);

  Renderer::Camera::Fov = 90.f;
  Renderer::Camera::Far = 300.f;

  // Renderer::Camera::Position = {0, 4, 0};
  // Renderer::Camera::Position = {-58.059, 0.6207, 3.102};
  // Renderer::Camera::Rotation = {0.0408, 1.7506, 0};

  Renderer::Camera::Position = {-38.4718, -1.1234, -1.3119};
  Renderer::Camera::Rotation = {0.6118, 1.6292, 0};

  Renderer::Config::ShowGBuffer = false;

  // uint64_t bunnyId =
  //    Renderer::CreateModelFromPrimitive(Renderer::EPrimitive::Cube);

  // uint64_t bunnyId = Renderer::CreateModelFromObj(
  //    std::string("../resources/models/bunny.obj"));
  // uint64_t armadilloId = Renderer::CreateModelFromObj(
  //    std::string("../resources/models/armadillo.obj"));

  // uint64_t sphereId = Renderer::CreateModelFromObj(
  //    std::string("../resources/models/sphere.obj"));

  Renderer::LoadMesh(std::string("../resources/models/sponza/sponza.obj"),
                     "sponza");
  Renderer::LoadMesh(std::string("../resources/models/cube.obj"), "cube");

  // Renderer::LoadTexture("../resources/textures/statue_head.jpg",
  // "statue_head");

  uint64_t mapId = Renderer::CreateModel("sponza");
  uint64_t cubeId = Renderer::CreateModel("cube");
  // uint64_t cubeId1 = Renderer::CreateModel(std::string("cube"));

  WindowHandler::CaptureMouse();

  Renderer::ModelInstance *cube =
      Renderer::Spawn(cubeId, {0.0f, -6.5f, 0.0f},
                      Renderer::Vec3::Zero()); // {100.0f, 100.0f, 100.0f});

  Renderer::ModelInstance *cube1 =
      Renderer::Spawn(cubeId, {5.0f, -6.5f, 0.0f},
                      Renderer::Vec3::Zero()); // {100.0f, 100.0f, 100.0f});

  cube1->color = Renderer::Color::Red;
  cube1->alpha = 0.1f;

  Renderer::ModelInstance *map =
      Renderer::Spawn(mapId, {0.0f, -6.5f, 0.0f}, Renderer::Vec3::Zero(),
                      {0.05f, 0.05f, 0.05f}); // {100.0f, 100.0f, 100.0f});

  // Point light
  // Renderer::Light *light =
  // Renderer::SpawnLight({ -10, 10, 0.0f }, {50, 50, 50}, 0);
  // light->lightType = Renderer::LightType::Point;
  // light->maxDist = 100;

  Renderer::Light *light;
  Renderer::Vec3 lightColor = {2.5, 2.5, 2.5};

  light = Renderer::SpawnLight({-30.83f, 0.109f, 6.87f}, lightColor, 0);
  light->lightType = Renderer::LightType::Point;
  light->maxDist = 100;

  light = Renderer::SpawnLight({-30.83f, 0.109f, -11.052f}, lightColor, 0);
  light->lightType = Renderer::LightType::Point;
  light->maxDist = 100;

  light = Renderer::SpawnLight({24.174f, 0.109f, -11.052f}, lightColor, 0);
  light->lightType = Renderer::LightType::Point;
  light->maxDist = 100;

  light = Renderer::SpawnLight({24.174f, 0.109f, 6.87f}, lightColor, 0);
  light->lightType = Renderer::LightType::Point;
  light->maxDist = 100;

  Renderer::Vec3 sunColor = {1.5f, 1.5f, 1.5f};

  light = Renderer::SpawnLight({1, -1, 1}, sunColor, 0);
  light->lightType = Renderer::LightType::Directional;

  light = Renderer::SpawnLight({-1, -1, -1}, sunColor, 0);
  light->lightType = Renderer::LightType::Directional;

  // Directional light
  // light = Renderer::SpawnLight(
  //   {0.2f, -1.0f, -0.3f}, Renderer::Color::White, 0);
  // light->lightType = Renderer::LightType::Directional;

  // Renderer::ModelInstance *sphere = Renderer::Spawn(
  //    sphereId, {-10, 10, 0.0f}, Renderer::Vec3::Zero(), {0.05f, 0.05f,
  //    0.05f});

  // Renderer::ModelInstance *inst0 = Renderer::Spawn(sphereId,
  // Renderer::Vec3::Zero(), Renderer::Vec3::Zero(), { 0.05f, 0.05f, 0.05f });
  // for (size_t i = 0; i < 100; ++i)
  // Renderer::Spawn(bunnyId, {i * 5.f, 0, 0});

  Renderer::Vec3 baseRot = {M_PI / 4.f, 0, 0};
  // Renderer::ModelInstance *inst1 =
  //    Renderer::Spawn(armadilloId, {3.f, 0.f, 0.f}, baseRot);
  // Renderer::ModelInstance *inst2 =
  //    Renderer::Spawn(armadilloId, {0.f, 3.f, 0.f}, baseRot);
  // Renderer::ModelInstance *inst3 = Renderer::Spawn(
  //    bunnyId, {0.f, 0.f, 3.f}, Renderer::Vec3::Zero(), {0.5f, 0.5f, 0.5f});

  // inst1->color = Renderer::Color::Red;
  // inst2->color = Renderer::Color::Green;
  // inst3->color = Renderer::Color::Blue;

  map->color = Renderer::Color::Grey;

  auto startTime = std::chrono::high_resolution_clock::now();
  auto frameTimestamp = startTime;

  while (WindowHandler::Update() && !esc) {
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
    UpdateInputs();

    ExtImGui::Update();

    auto currentTime = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float, std::chrono::seconds::period>(
                     currentTime - startTime)
                     .count();

    frameTime = std::chrono::duration<float, std::chrono::seconds::period>(
                    currentTime - frameTimestamp)
                    .count();

    frameTimestamp = currentTime;

    // inst0->SetRotation({time * 0.5f * static_cast<float>(M_PI) / 2.f,
    //                    time * static_cast<float>(M_PI) / 2.f,
    //                    time * static_cast<float>(M_PI)});
    // inst1->SetRotation(
    //    {baseRot.x + time * static_cast<float>(M_PI) / 2.f, 0, 0});
    // inst2->SetRotation(
    //    {0, baseRot.y + time * static_cast<float>(M_PI) / 2.f, 0});
    // inst3->SetRotation(
    //    {0, baseRot.z + time * static_cast<float>(M_PI) / 2.f, 0});

    ////light->vector.x = sinf(time * 2.0f) * 10.0f;
    // sphere->SetPosition({light->vector.x, 10.0f, 0.0f});

    // Renderer::Camera::Rotation = {
    //    0, baseRot.x + time / 2.f * static_cast<float>(M_PI) / 2.f, 0};
  }

  ExtImGui::Shutdown();
  Renderer::Shutdown();
  WindowHandler::Shutdown();

  return 0;
}
