#ifndef PARESEGCODE_1595984973500_H
#define PARESEGCODE_1595984973500_H

#include "gcode/interface.h"
#include "gcodeprocesslib/header.h"

#include <unordered_map>

namespace gcode
{
    void GCODE_API paraseGcode(const std::string& gCodeFile, std::vector<std::vector<SliceLine3D>>& m_sliceLayers, trimesh::box3& box, std::unordered_map<std::string, std::string>& kvs);
}
#endif // PARESEGCODE_1595984973500_H