#include <Eigen/Core>

#include <array>
#include <chrono>
#include <unordered_map>
#include <vector>

#include "configuration_helper.hpp"

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

/*template <>
struct hash<std::array<float, 5>>
{
  size_t operator()(const std::array<float, 5>& v) const
  {
    size_t h = std::hash<float>()(v[0]);
    hash_combine(h, v[1]);
    hash_combine(h, v[2]);
    hash_combine(h, v[3]);
    hash_combine(h, v[4]);
    return h;
  }
};

template <>
struct hash<std::array<float, 12>>
{
  size_t operator()(const std::array<float, 12>& v) const
  {
    size_t h = std::hash<float>()(v[0]);
    hash_combine(h, v[1]);
    hash_combine(h, v[2]);
    hash_combine(h, v[3]);
    hash_combine(h, v[4]);
    hash_combine(h, v[5]);
    hash_combine(h, v[6]);
    hash_combine(h, v[7]);
    hash_combine(h, v[8]);
    hash_combine(h, v[9]);
    hash_combine(h, v[10]);
    hash_combine(h, v[11]);
    return h;
  }
};*/

} // namespace std

namespace ObjLoader {
template <typename Vertex> struct InternalMeshLoadData {
  std::vector<Eigen::Vector3f> positions;
  std::vector<Eigen::Vector3f> normals;
  std::vector<Vertex> vertices;
  std::unordered_map<Vertex, VERTEX_INDICES_TYPE> verticesSet;
  std::vector<VERTEX_INDICES_TYPE> indices;
  VERTEX_INDICES_TYPE higestIndex = 0;
};

template <typename Vertex>
bool LoadObj(const std::string &filename, std::vector<Vertex> &vertexBuffer,
             std::vector<VERTEX_INDICES_TYPE> &indexBuffer) {
  InternalMeshLoadData<Vertex> loadData{};

  /*
    vertices count: 1662427
    indices count: 5626896
  */
  vertexBuffer.reserve(1662427);
  indexBuffer.reserve(5626896);

  std::ifstream file(filename);

  if (file.good() == false) {
    std::cout << "File not found " << filename << std::endl;
    return false;
  }

  std::cout << "\nLoading " << filename << "\nPlease wait..." << std::endl;

  auto start = std::chrono::system_clock::now();

  std::string line;
  while (std::getline(file, line)) {
    if (line.length() == 0)
      continue;

    // std::cout << line << std::endl;

    if (!ProcessLine<Vertex>(std::istringstream(line), loadData)) {
      std::cout << "Invalid file \"" << filename << "\" (" << line << ")"
                << std::endl;
      return false;
    }
  }

  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;

  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
  std::cout << "vertices count: " << loadData.vertices.size() << "\n";
  std::cout << "indices count: " << loadData.indices.size() << "\n";

  // std::swap(mesh.GetVertices(), loadData.vertices);
  // std::swap(mesh.GetIndices(), loadData.indices);

  vertexBuffer = std::move(loadData.vertices);
  /*for (Vertex& vertex : loadData.vertices)
  {
    for (uint64_t i = 0; i < sizeof(Vertex); i++)
    {
      vertexBuffer.push_back(vertex.data[i]);
    }
  }*/

  // mesh.GetVertices().resize(loadData.vertices.size() * Vertex::Size);
  // memcpy(mesh.GetVertices().data(), loadData.vertices.data(),
  // loadData.vertices.size() * Vertex::Size * sizeof(float));

  // mesh.GetVertices().insert(mesh.GetVertices().begin(),
  // loadData.vertices.begin(), loadData.vertices.end());
  // mesh.GetIndices().insert(mesh.GetIndices().begin(),
  // loadData.indices.begin(), loadData.indices.end());
  indexBuffer = std::move(loadData.indices);

  // for (uint64_t i = 0; i < loadData.indices.size(); i++)
  //{
  //  mesh.GetIndices().push_back(loadData.indices[i]);
  //}

  return true;
}

template <typename Vertex>
bool ProcessLine(std::istringstream &&line,
                 InternalMeshLoadData<Vertex> &loadData) noexcept {
  constexpr bool useIndices = true;
  constexpr bool loadNormals = true;
  constexpr bool loadTextureUVs = false;
  constexpr bool computeTangents = false;
  constexpr bool normalizeNormals = true;

  // constexpr bool useIndices = (params &
  // MeshLoadStaticParamsBit::DontUseIndices) == MeshLoadStaticParamsBit::None;
  // constexpr bool loadNormals = (params &
  // MeshLoadStaticParamsBit::DontLoadNormals) == MeshLoadStaticParamsBit::None;
  // constexpr bool loadTextureUVs = (params &
  // MeshLoadStaticParamsBit::DontLoadTextureUVs) ==
  // MeshLoadStaticParamsBit::None; constexpr bool computeTangents = (params &
  // MeshLoadStaticParamsBit::ComputeTangents) != MeshLoadStaticParamsBit::None;
  // constexpr bool normalizeNormals = (params &
  // MeshLoadStaticParamsBit::NormalizeNormals) !=
  // MeshLoadStaticParamsBit::None;

  std::string firstToken;

  if (!(line >> firstToken))
    return false;

  if (firstToken[0] == '#')
    return true;

  if (firstToken == "v") {
    return ProcessPosition<Vertex>(std::forward<std::istringstream>(line),
                                   loadData);
  } else if (firstToken == "vt") {
    // if constexpr (loadTextureUVs)
    //  return ProcessTextureUV<Vertex>(std::forward<std::istringstream>(line),
    //                                  loadData);
    // else
    return true;
  } else if (firstToken == "vn") {
    if constexpr (loadNormals)
      return ProcessNormal<Vertex>(std::forward<std::istringstream>(line),
                                   loadData);
    else
      return true;
  } else if (firstToken == "vp") {
    return false;
  } else if (firstToken == "f") {
    return ProcessFace<Vertex>(std::forward<std::istringstream>(line),
                               loadData);
  } else if (firstToken == "g") {
    return true;
  } else if (firstToken == "o") {
    return true;
  } else if (firstToken == "s") {
    return true;
  } else if (firstToken == "mtllib") {
    return true;
  } else if (firstToken == "usemtl") {
    return true;
  }

  return false;
}

template <typename Vertex>
bool ProcessPosition(std::istringstream &&line,
                     InternalMeshLoadData<Vertex> &loadData) noexcept {
  Eigen::Vector3f pos;
  if (!(line >> pos[0] >> pos[1] >> pos[2]))
    return false;

  // if constexpr ((params & MeshLoadStaticParamsBit::InvertY) !=
  // MeshLoadStaticParamsBit::None) pos.y = -pos.y;
  // pos = -pos;

  loadData.positions.push_back(pos);

  return true;
}

// template <typename Vertex>
// bool ProcessTextureUV(std::istringstream&& line,
// InternalMeshLoadData<Vertex>& loadData) noexcept
//{
//  Eigen::Vector2f uv;
//  if (!(line >> uv.x >> uv.y))
//    return false;
//
//  loadData.textureUVs.push_back(uv);
//
//  return true;
//}

template <typename Vertex>
bool ProcessNormal(std::istringstream &&line,
                   InternalMeshLoadData<Vertex> &loadData) noexcept {
  Eigen::Vector3f normal;
  if (!(line >> normal[0] >> normal[1] >> normal[2]))
    return false;

  loadData.normals.push_back(normal);

  return true;
}

int64_t
GetNextFaceIndex(std::string &faceToken) // TODO modifying string is heavy use
                                         // const char*& instead
{
  if (faceToken[0] == '/')
  {
    faceToken.erase(0, faceToken.find('/') + 1);
    return 0;
  }

  int64_t index =
      std::stoll(faceToken); // stoll stops when it encounter a non numeric
                             // character ("42/666/-1" -> 42)
  faceToken.erase(0, faceToken.find('/') + 1);

  return index;
}

template <typename Vertex>
bool ProcessFace(std::istringstream &&line,
                 InternalMeshLoadData<Vertex> &loadData) noexcept {
  constexpr bool useIndices = true;
  constexpr bool loadNormals = true;
  constexpr bool loadTextureUVs = false;
  constexpr bool computeTangents = false;
  constexpr bool normalizeNormals = true;

  // constexpr bool useIndices = (params &
  // MeshLoadStaticParamsBit::DontUseIndices) == MeshLoadStaticParamsBit::None;
  // constexpr bool loadNormals = (params &
  // MeshLoadStaticParamsBit::DontLoadNormals) == MeshLoadStaticParamsBit::None;
  // constexpr bool loadTextureUVs = (params &
  // MeshLoadStaticParamsBit::DontLoadTextureUVs) ==
  // MeshLoadStaticParamsBit::None; constexpr bool computeTangents = (params &
  // MeshLoadStaticParamsBit::ComputeTangents) != MeshLoadStaticParamsBit::None;
  // constexpr bool normalizeNormals = (params &
  // MeshLoadStaticParamsBit::NormalizeNormals) !=
  // MeshLoadStaticParamsBit::None;

  std::vector<Vertex> polygonVertices;
  std::string nextToken;

  while (line >> nextToken) {
    Vertex vertex;

    int64_t index = GetNextFaceIndex(nextToken);

    if (index == 0)
      return false;

    if (index < 0)
      index = int64_t(loadData.positions.size()) + index;
    else if (index > 0)
      index--;

    vertex.pos.members[0] = loadData.positions[index][0];
    vertex.pos.members[1] = loadData.positions[index][1];
    vertex.pos.members[2] = loadData.positions[index][2];

    if constexpr (loadTextureUVs) {
      /*index = GetNextFaceIndex(nextToken);

      if (index != 0) {
        if (index < 0)
          index = int64(loadData.textureUVs.size()) + index;
        else if (index > 0)
          index--;

        vertex.Get<TextureUV>() = loadData.textureUVs[index];
      }*/
    } else
      GetNextFaceIndex(nextToken);

    if constexpr (loadNormals) {
      index = GetNextFaceIndex(nextToken);

      if (index != 0) {
        if (index < 0)
          index = int64_t(loadData.normals.size()) + index;
        else if (index > 0)
          index--;

        if (loadData.normals.size() == 0) {
          // vertex.Get<NormalXYZ>() = { 1, 0, 0 };
          vertex.nor.members[0] = 1;
          vertex.nor.members[1] = 0;
          vertex.nor.members[2] = 0;
        } else {
          if constexpr (normalizeNormals) {
            // vertex.Get<NormalXYZ>() =
            // loadData.normals[index].GetNormalized();
            auto normalized = loadData.normals[index].normalized();

            vertex.nor.members[0] = normalized[0];
            vertex.nor.members[1] = normalized[1];
            vertex.nor.members[2] = normalized[2];
          } else {
            // vertex.Get<NormalXYZ>() = loadData.normals[index];
            vertex.nor.members[0] = loadData.normals[index][0];
            vertex.nor.members[1] = loadData.normals[index][1];
            vertex.nor.members[2] = loadData.normals[index][2];
          }
        }
      }
    }

    polygonVertices.push_back(vertex);
  }

  if (polygonVertices.size() < 3)
    return false;
  else if (polygonVertices.size() > 3) {
    InsertVertex<Vertex>(polygonVertices[0], loadData);
    InsertVertex<Vertex>(polygonVertices[2], loadData);
    InsertVertex<Vertex>(polygonVertices[3], loadData);
  }

  InsertVertex<Vertex>(polygonVertices[0], loadData);
  InsertVertex<Vertex>(polygonVertices[1], loadData);
  InsertVertex<Vertex>(polygonVertices[2], loadData);

  return true;
}

template <typename Vertex>
void InsertVertex(const Vertex &vertex,
                  InternalMeshLoadData<Vertex> &loadData) {
  constexpr bool useIndices = true;
  // constexpr bool useIndices = (params &
  // MeshLoadStaticParamsBit::DontUseIndices) == MeshLoadStaticParamsBit::None;

  if constexpr (useIndices == false) {
    loadData.indices.push_back(loadData.indices.size());
    loadData.vertices.push_back(vertex);
    return;
  }

  auto search = loadData.verticesSet.find(vertex);
  if (search != loadData.verticesSet.end()) {
    loadData.indices.push_back(search->second);
    return;
  }

  loadData.indices.push_back(loadData.higestIndex);

  loadData.vertices.push_back(vertex);
  loadData.verticesSet.insert({vertex, loadData.higestIndex++});
}
} // namespace ObjLoader
