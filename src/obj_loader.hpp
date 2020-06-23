#pragma once

#include "data_structures.hpp"

namespace ObjLoader {

struct ShapeData {
  // VERTEX_INDICES_TYPE indexCount; //Currently not used
  VERTEX_INDICES_TYPE indexOffset;
  std::vector<std::string> diffuseMaps;
  std::vector<Eigen::Vector3f> diffuseColors;
  std::vector<std::string> alphaMaps;
};

void LoadObj(const std::string &filename,
             std::vector<LiteralVertex> &vertexBuffer,
             std::vector<VERTEX_INDICES_TYPE> &indexBuffer,
             std::vector<ShapeData> &shapeData);

} // namespace ObjLoader
