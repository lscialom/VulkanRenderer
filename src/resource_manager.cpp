#include "resource_manager.hpp"

#include <unordered_map>

#define DEFAULT_TEXTURE_NAME "default_texture"
#define MISSING_TEXTURE_NAME "missing_texture"

namespace Renderer {
namespace ResourceManager {

static std::unordered_map<std::string, Texture *> textures;

void Init() {

  LoadTexture(RESOURCES_PATH + "textures/default_texture.jpg",
              DEFAULT_TEXTURE_NAME);
  LoadTexture(RESOURCES_PATH + "textures/missing_texture.png",
              MISSING_TEXTURE_NAME);
}

void Shutdown() {
  for (const auto &pair : textures)
    delete pair.second;

  textures.clear();
}

void LoadTexture(const std::string &texturePath, const std::string &texName) {

  if (textures.find(texName) != textures.end()) {

    std::cout << "Skipping texture " << texName
              << " because another texture (or the same) has already been "
                 "registered with that name."
              << std::endl;

    return;
  }

  textures[texName] = new Texture();
  if (!textures[texName]->init(texturePath)) {
    DestroyTexture(texName);
    return;
  }

  std::cout << "Loaded texture " << texName << std::endl;
}

void DestroyTexture(const std::string &texName) {

  auto it = textures.find(texName);

  if (it != textures.end()) {

    delete textures[texName];
    textures.erase(it);

    std::cout << "Destroyed texture " << texName << std::endl;
  }
}

Texture *GetTexture(const std::string &textureName) {
  auto it = textures.find(textureName);

  if (it == textures.end()) {

    std::cout << "Could not find any texture named " << textureName
              << std::endl;

    return textures[MISSING_TEXTURE_NAME];
  }

  return it->second;
}

} // namespace ResourceManager
} // namespace Renderer
