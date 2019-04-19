#include "obj_loader.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tinyobjloader/tiny_obj_loader.h>

template <class T> inline void hash_combine(std::size_t &seed, const T &v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  // seed ^= hasher(v) + 0x11111111 + (seed << 6) + (seed >> 2);
}

namespace std {
template <> struct hash<std::array<float, 3>> {
  size_t operator()(const std::array<float, 3> &v) const {
    size_t h = std::hash<float>()(v[0]);
    hash_combine(h, v[1]);
    hash_combine(h, v[2]);
    return h;
  }
};

template <> struct hash<LiteralVertex> {
  size_t operator()(const LiteralVertex &v) const {
    std::array<float, 3> pos;
    std::array<float, 3> nor;

    pos[0] = v.pos.members[0];
    pos[1] = v.pos.members[1];
    pos[2] = v.pos.members[2];

    nor[0] = v.nor.members[0];
    nor[1] = v.nor.members[1];
    nor[2] = v.nor.members[2];

    size_t h = std::hash<std::array<float, 3>>()(pos);
    hash_combine(h, nor);
    return h;
  }
};
} // namespace std

namespace ObjLoader {

void LoadObj(const std::string &filename,
             std::vector<LiteralVertex> &vertexBuffer,
             std::vector<VERTEX_INDICES_TYPE> &indexBuffer) {

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn, err;

  std::unordered_map<LiteralVertex, uint32_t> uniqueVertices = {};

  std::cout << "\nLoading " << filename << "\nPlease wait..." << std::endl;

  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                        filename.c_str())) {
    throw std::runtime_error(warn + err);
  }

  for (const auto &shape : shapes) {
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
  }

  std::cout << "Loading complete !" << std::endl;
}

} // namespace ObjLoader
