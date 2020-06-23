#include "resource_manager.hpp"
#include "obj_loader.hpp"

#include "mesh.hpp"

#include <unordered_map>

namespace Renderer {
namespace ResourceManager {

static std::unordered_map<std::string, Texture *> textures;
static std::unordered_map<std::string, Mesh *> meshes;

void Init() {

  LoadTexture(RESOURCES_PATH + "textures/default_texture.jpg",
              DEFAULT_TEXTURE_NAME, TextureUsage::Color, true);
  textures[DEFAULT_TEXTURE_NAME_ALT] = textures[DEFAULT_TEXTURE_NAME];

  LoadTexture(RESOURCES_PATH + "textures/missing_texture.png",
              MISSING_TEXTURE_NAME, TextureUsage::Color, true);
}

void Shutdown() {
  textures.erase(
      DEFAULT_TEXTURE_NAME_ALT); // Same pointer as DEFAULT_TEXTURE_NAME, so
                                 // this keeps the program from freeing the same
                                 // pointer twice

  for (const auto &pair : textures)
    delete pair.second;

  textures.clear();

  for (const auto &pair : meshes)
    delete pair.second;

  meshes.clear();
}

void LoadTexture(const std::string &texturePath, const std::string &texName,
                 TextureUsage textureUsage, bool mipMap) {

  if (textures.find(texName) != textures.end()) {

    // std::cout << "Skipping texture " << texName
    //          << " because another texture (or the same) has already been "
    //             "registered with that name."
    //          << std::endl;

    return;
  }

  textures[texName] = new Texture();
  if (!textures[texName]->init(texturePath, textureUsage, mipMap)) {
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

void LoadMesh(const std::string &meshPath, const std::string &meshName) {

  if (meshes.find(meshName) != meshes.end()) {

    std::cout << "Skipping mesh " << meshName
              << " because another mesh (or the same) has already been "
                 "registered with that name."
              << std::endl;

    return;
  }

  Mesh *mesh = new Mesh();

  std::vector<LiteralVertex> vertices;
  std::vector<VERTEX_INDICES_TYPE> indices;
  std::vector<ObjLoader::ShapeData> shapeData;

  ObjLoader::LoadObj(meshPath, vertices, indices, shapeData);

  mesh->init(vertices.begin(), vertices.end(), indices.begin(), indices.end(),
             shapeData.begin(), shapeData.end());

  meshes[meshName] = mesh;
}

void DestroyMesh(const std::string &meshName) {

  auto it = meshes.find(meshName);

  if (it != meshes.end()) {

    delete meshes[meshName];
    meshes.erase(it);

    std::cout << "Destroyed mesh " << meshName << std::endl;
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

Mesh *GetMesh(const std::string &meshName) {
  auto it = meshes.find(meshName);

  if (it == meshes.end()) {

    std::cout << "Could not find any mesh named " << meshName << std::endl;

    return nullptr;
  }

  return it->second;
}

} // namespace ResourceManager
} // namespace Renderer
