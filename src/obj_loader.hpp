#include "data_structures.hpp"

namespace ObjLoader {

void LoadObj(const std::string &filename,
             std::vector<LiteralVertex> &vertexBuffer,
             std::vector<VERTEX_INDICES_TYPE> &indexBuffer);

} // namespace ObjLoader
