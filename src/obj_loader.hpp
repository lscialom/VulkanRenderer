#include "data_structures.hpp"

namespace ObjLoader {

struct ShapeData {
  VERTEX_INDICES_TYPE indexCount;
};

void LoadObj(const std::string &filename,
             std::vector<LiteralVertex> &vertexBuffer,
             std::vector<VERTEX_INDICES_TYPE> &indexBuffer,
             std::vector<ShapeData> &shapeData);

} // namespace ObjLoader
