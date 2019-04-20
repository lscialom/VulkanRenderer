#include "texture.hpp"

#define RESOURCES_PATH std::string("../resources/")

namespace Renderer {
namespace ResourceManager {

void Init();
void Shutdown();

void LoadTexture(const std::string &texturePath, const std::string &texName);
void DestroyTexture(const std::string &texName);

Texture* GetTexture(const std::string& textureName);

} // namespace ResourceManager
} // namespace Renderer
