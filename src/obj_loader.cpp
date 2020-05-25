#include "obj_loader.hpp"

#include "resource_manager.hpp"

#include "texture.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tinyobjloader/tiny_obj_loader.h>

template <class T> inline void hash_combine(std::size_t &seed, const T &v) {
  std::hash<std::decay_t<T>> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  // seed ^= hasher(v) + 0x11111111 + (seed << 6) + (seed >> 2);
}

namespace std {

template <size_t N> struct hash<LiteralVector<N>> {

  static_assert(N > 0);

  size_t operator()(const LiteralVector<N> &v) const {

    size_t h = std::hash<std::decay_t<decltype(v.members[0])>>()(v.members[0]);

    for (size_t i = 1; i < N; ++i)
      hash_combine(h, v.members[i]);

    return h;
  }
};

template <> struct hash<LiteralVertex> {
  size_t operator()(const LiteralVertex &v) const {

    size_t h = std::hash<std::decay_t<decltype(v.pos)>>()(v.pos);

    hash_combine(h, v.nor);
    hash_combine(h, v.texcoords);

    return h;
  }
};
} // namespace std

namespace ObjLoader {

void LoadObj(const std::string &filename,
             std::vector<LiteralVertex> &vertexBuffer,
             std::vector<VERTEX_INDICES_TYPE> &indexBuffer,
             std::vector<ShapeData> &shapeData) {

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn, err;

  std::unordered_map<LiteralVertex, uint32_t> uniqueVertices = {};

  std::cout << "\nLoading " << filename << "\nPlease wait..." << std::endl;

  size_t cwd_npos = filename.rfind("/");
  if (cwd_npos == std::string::npos)
    cwd_npos = filename.rfind("\\");

  std::string cwd = "";
  if (cwd_npos != std::string::npos)
    cwd = filename.substr(0, cwd_npos + 1);

  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                        filename.c_str(), cwd.c_str())) {
    throw std::runtime_error(warn + err);
  }

  VERTEX_INDICES_TYPE indexOffset = 0;

  for (const auto &shape : shapes) {

    std::cout << "Loading " << shape.name << std::endl;

    for (const auto &index : shape.mesh.indices) {
      LiteralVertex vertex = {};

      vertex.pos = {attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]};

      vertex.nor = {attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2]};

      if (!attrib.texcoords.empty()) {
        vertex.texcoords = {attrib.texcoords[2 * index.texcoord_index + 0],
                            1.0f -
                                attrib.texcoords[2 * index.texcoord_index + 1]};
      }

      if (uniqueVertices.count(vertex) == 0) {
        uniqueVertices[vertex] = static_cast<uint32_t>(vertexBuffer.size());
        vertexBuffer.push_back(vertex);
      }

      indexBuffer.push_back(uniqueVertices[vertex]);
    }

    ShapeData data;
    data.diffuseMaps =
        std::vector(shape.mesh.material_ids.size(), std::string());
    data.alphaMaps = std::vector(shape.mesh.material_ids.size(), std::string());

    for (size_t index = 0; index < shape.mesh.material_ids.size(); ++index) {

      int matId = shape.mesh.material_ids[index];

      if (matId < 0)
        continue;

      if (!materials[matId].alpha_texname.empty()) {
        std::string texPath = cwd + materials[matId].alpha_texname;
        Renderer::ResourceManager::LoadTexture(texPath, texPath,
                                               Renderer::TextureUsage::Data);

        data.alphaMaps[index] = texPath;
      }

      if (!materials[matId].diffuse_texname.empty()) {
        std::string texPath = cwd + materials[matId].diffuse_texname;
        Renderer::ResourceManager::LoadTexture(texPath, texPath,
                                               Renderer::TextureUsage::Color);

        data.diffuseMaps[index] = texPath;
      }
    }

    // data.indexCount = static_cast<uint32_t>(shape.mesh.indices.size());
    // //ShapeData::indexCount currently not in use

    data.indexOffset = indexOffset;

    shapeData.push_back(data);

    indexOffset += static_cast<uint32_t>(shape.mesh.indices.size());
  }

  std::cout << "Loading complete !" << std::endl;
}

} // namespace ObjLoader
