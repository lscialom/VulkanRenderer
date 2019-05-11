#include "texture.hpp"
#include "mesh.hpp"

#define RESOURCES_PATH std::string("../resources/")

#define DEFAULT_TEXTURE_NAME "default_texture"
#define MISSING_TEXTURE_NAME "missing_texture"

namespace Renderer {
namespace ResourceManager {

void Init();
void Shutdown();

void LoadTexture(const std::string &texturePath, const std::string &texName);
void DestroyTexture(const std::string &texName);

Texture* GetTexture(const std::string& textureName);

void LoadMesh(const std::string& meshPath, const std::string& meshName);
void DestroyMesh(const std::string& meshName);

Mesh* GetMesh(const std::string& meshName);

} // namespace ResourceManager
} // namespace Renderer
