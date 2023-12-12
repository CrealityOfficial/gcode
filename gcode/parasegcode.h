#ifndef PARESEGCODE_1595984973500_H
#define PARESEGCODE_1595984973500_H

#include "gcode/interface.h"
#include "gcodeprocesslib/header.h"
#include "gcodeprocesslib/gcode_position.h"
#include <unordered_map>

namespace gcode
{
    gcode_position_args get_args_(bool g90_g91_influences_extruder, int buffer_size);
    void Stringsplit(std::string str, const char split, std::vector<std::string>& res);
    void GCODE_API paraseGcode(const std::string& gCodeFile, std::vector<std::vector<SliceLine3D>>& m_sliceLayers, trimesh::box3& box, std::unordered_map<std::string, std::string>& kvs);
}
#endif // PARESEGCODE_1595984973500_H