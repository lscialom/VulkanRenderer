#pragma once

#include <string>

#define RESOURCES_PATH std::string("../resources/")

#define DEFAULT_TEXTURE_NAME "default_texture"
#define DEFAULT_TEXTURE_NAME_ALT ""
#define MISSING_TEXTURE_NAME "missing_texture"

namespace Renderer {

enum class TextureUsage : uint8_t;

struct Texture;
struct Mesh;

namespace ResourceManager {

void Init();
void Shutdown();

void LoadTexture(const std::string &texturePath, const std::string &texName,
                 TextureUsage textureUsage);
void DestroyTexture(const std::string &texName);

Texture *GetTexture(const std::string &textureName);

void LoadMesh(const std::string &meshPath, const std::string &meshName);
void DestroyMesh(const std::string &meshName);

Mesh *GetMesh(const std::string &meshName);

} // namespace ResourceManager
} // namespace Renderer
