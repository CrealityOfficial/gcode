#include <sstream>
#include <fstream>

#include "gcode/parasegcode.h"
#include "gcodeprocesslib/gcode_position.h"
#include "gcodeprocesslib/gcode_parser.h"

#define DEFAULT_GCODE_BUFFER_SIZE 50
#define DEFAULT_G90_G91_INFLUENCES_EXTRUDER false

namespace gcode
{
    gcode_position_args get_args_(bool g90_g91_influences_extruder, int buffer_size)
    {
        gcode_position_args args;
        // Configure gcode_position_args
        args.g90_influences_extruder = g90_g91_influences_extruder;
        args.position_buffer_size = buffer_size;
        args.autodetect_position = true;
        args.home_x = 0;
        args.home_x_none = true;
        args.home_y = 0;
        args.home_y_none = true;
        args.home_z = 0;
        args.home_z_none = true;
        args.shared_extruder = true;
        args.zero_based_extruder = true;


        args.default_extruder = 0;
        args.xyz_axis_default_mode = "absolute";
        args.e_axis_default_mode = "absolute";
        args.units_default = "millimeters";
        args.location_detection_commands = std::vector<std::string>();
        args.is_bound_ = false;
        args.is_circular_bed = false;
        args.x_min = -9999;
        args.x_max = 9999;
        args.y_min = -9999;
        args.y_max = 9999;
        args.z_min = -9999;
        args.z_max = 9999;
        return args;
    }
    
    void Stringsplit(std::string str, const char split, std::vector<std::string>& res)
    {
        std::istringstream iss(str);	// 输入流
        std::string token;			// 接收缓冲区
        while (getline(iss, token, split))	// 以split为分隔符
        {
            res.push_back(token);
        }
    }

    void removeSpace(std::string& str)
    {
        str.erase(0, str.find_first_not_of(" "));
        str.erase(str.find_last_not_of(" ") + 1);
    }

    void changeKey(const std::string& cource, const std::string& dest, std::unordered_map<std::string, std::string>& kvs)
    {
        auto iter = kvs.find(cource);
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, ';', _kvs);
            if (_kvs.size() > 1) {
                iter->second = _kvs[0];
            }
            kvs.insert(std::make_pair(dest, iter->second));
            kvs.erase(iter);
        }
    }

    void getKvs(const std::string& comment, const SliceCompany& sliceCompany, std::unordered_map<std::string, std::string>& kvs)
    {
        //收集所有的参数信息
        std::vector<std::string> _kvs;
        switch (sliceCompany)
        {
        case SliceCompany::prusa:
            Stringsplit(comment, ':', _kvs);
            if (_kvs.size() > 1) {
                removeSpace(_kvs[0]);
                removeSpace(_kvs[1]);
                kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
            }
            else { //end of gcode
                _kvs.clear();
                Stringsplit(comment, '=', _kvs);
                if (_kvs.size() > 1) {
                    removeSpace(_kvs[0]);
                    removeSpace(_kvs[1]);
                    kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
                }
            }
            break;
        case SliceCompany::bambu:
        case SliceCompany::superslicer:
            Stringsplit(comment, '=', _kvs);
            if (_kvs.size() > 1) {
                removeSpace(_kvs[0]);
                removeSpace(_kvs[1]);
                kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
            }
            else { //end of gcode
                _kvs.clear();
                Stringsplit(comment, ':', _kvs);
                if (_kvs.size() > 1) {
                    removeSpace(_kvs[0]);
                    removeSpace(_kvs[1]);
                    kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
                }
            }
            break;
        case SliceCompany::simplify:
            Stringsplit(comment, ',', _kvs);
            if (_kvs.size() > 1) {
                removeSpace(_kvs[0]);
                removeSpace(_kvs[1]);
                kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
            }
            else { //end of gcode
                _kvs.clear();
                Stringsplit(comment, ':', _kvs);
                if (_kvs.size() > 1) {
                    removeSpace(_kvs[0]);
                    removeSpace(_kvs[1]);
                    kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
                }
            }
            break;
        default:
            Stringsplit(comment, ':', _kvs);
            if (_kvs.size() > 1) {
                removeSpace(_kvs[0]);
                removeSpace(_kvs[1]);
                kvs.insert(std::make_pair(_kvs[0], _kvs[1]));
            }
            break;
        };
    }

    void _paraseGcode(SliceCompany& sliceCompany,const std::string& gCodeFile, trimesh::box3& box, std::vector<std::vector<SliceLine3D>>& m_sliceLayers, std::unordered_map<std::string, std::string>& kvs)
    {
        //SliceCompany sliceCompany = SliceCompany::creality;
        //trimesh::box3 box;
        std::vector<SliceLine3D > m_sliceLines;
        const char* path = gCodeFile.data();
        //sphere
        gcode_parser parser_;
        std::ifstream gcode_file;
        gcode_file.open(path);

        std::string line;
        int lines_with_no_commands = 0;
        gcode_file.sync_with_stdio(false);

        gcode_parser parser;
        int gcodes_processed = 0;
        int num_arc_commands_ = 0;
        int lines_processed_ = 0;
        bool is_shell = true;
        gcode_position* p_source_position_ = new gcode_position(get_args_(DEFAULT_G90_G91_INFLUENCES_EXTRUDER, DEFAULT_GCODE_BUFFER_SIZE));

        if (gcode_file.is_open())
        {
            parsed_command cmd;
            bool is_get_company = false;
            while (std::getline(gcode_file, line))
            {
                lines_processed_++;
                bool isLayer = false;
                cmd.clear();
                parser.try_parse_gcode(line.c_str(), cmd);
                bool has_gcode = false;
                if (cmd.gcode.length() > 0)
                {
                    has_gcode = true;
                    gcodes_processed++;
                }
                else
                {
                    lines_with_no_commands++;
                }
                p_source_position_->update(cmd, lines_processed_, gcodes_processed, -1);
                gcode_comment_processor* gcode_comment_processor = p_source_position_->get_gcode_comment_processor();

                getKvs(cmd.comment, sliceCompany, kvs);

                if (!is_get_company)
                {
                    std::vector<std::string> res;
                    Stringsplit(cmd.comment, ' ', res);

                    if(!res.empty())
                        is_get_company = true;
                    for (std::string t : res)
                    {

                        if (t == "Creality")
                        {
                            sliceCompany = SliceCompany::creality;
                            break;
                        }
                        else if (t == "PrusaSlicer")
                        {
                            sliceCompany = SliceCompany::prusa;
                            break;
                        }
                        else if (t == "BambuStudio")
                        {
                            sliceCompany = SliceCompany::bambu;
                            break;
                        }
                        else if (t == "Cura_SteamEngine")
                        {
                            sliceCompany = SliceCompany::cura;
                            break;
                        }
                        else if (t == "SuperSlicer")
                        {
                            sliceCompany = SliceCompany::superslicer;
                            break;
                        }
                        else if (t == "ideaMaker")
                        {
                            sliceCompany = SliceCompany::ideamaker;
                            break;
                        }
                        else if (t == "ffslicer")
                        {
                            sliceCompany = SliceCompany::ffslicer;
                            break;
                        }
                        else if (t == "Simplify3D(R)")
                        {
                            sliceCompany = SliceCompany::simplify;
                            break;
                        }
                        else
                            is_get_company = false;
                    }
                }


                if (sliceCompany == SliceCompany::creality && (cmd.comment == "TYPE:SKIRT" || cmd.comment == "TYPE:SUPPORT-INTERFACE" || cmd.comment == "TYPE:SUPPORT" ||
                    cmd.comment == "TYPE:FILL" || cmd.comment == "TYPE:WALL-INNER"))
                {
                    is_shell = false; continue;
                }
                else if (sliceCompany == SliceCompany::creality && (cmd.comment == "TYPE:SKIN" || cmd.comment == "TYPE:WALL-OUTER"))
                    is_shell = true;

                if (sliceCompany == SliceCompany::prusa && (cmd.comment == "TYPE:Solid infill" || cmd.comment == "TYPE:External perimeter"))
                    is_shell = true;
                else if (sliceCompany == SliceCompany::prusa && (cmd.comment == "WIPE_START" || cmd.comment == "WIPE_END"))
                {
                    is_shell = false;
                    continue;
                }
                if (sliceCompany == SliceCompany::bambu && (cmd.comment == " FEATURE: Internal solid infill" || cmd.comment == " FEATURE: Outer wall"))
                    is_shell = true;
                else if (sliceCompany == SliceCompany::bambu && (cmd.comment == " WIPE_START" || cmd.comment == " WIPE_END"))
                {
                    is_shell = false;
                    continue;
                }

                if (sliceCompany == SliceCompany::simplify && (cmd.comment == " inner perimeter" ||
                    cmd.comment == " infill" || cmd.comment == " skirt"))
                {
                    is_shell = false; continue;
                }
                else
                    is_shell = true;


                if ((sliceCompany == SliceCompany::creality || sliceCompany == SliceCompany::cura || sliceCompany == SliceCompany::ideamaker) && cmd.comment.find("LAYER:") != std::string::npos)
                {
                    isLayer = true;
                }
                else if (sliceCompany == SliceCompany::ffslicer && cmd.comment.find("layer:") != std::string::npos)
                {
                    isLayer = true;
                }
                else if ((sliceCompany == SliceCompany::prusa || sliceCompany == SliceCompany::superslicer) && cmd.comment.find("AFTER_LAYER_CHANGE") != std::string::npos)
                {
                    isLayer = true;
                }
                else if (sliceCompany == SliceCompany::bambu && cmd.comment.find(" CHANGE_LAYER") != std::string::npos)
                {
                    isLayer = true;
                }
                else if (sliceCompany == SliceCompany::simplify && cmd.comment.find("layer") != std::string::npos)
                {
                    isLayer = true;
                }

                if ((sliceCompany == SliceCompany::creality || sliceCompany == SliceCompany::cura || sliceCompany == SliceCompany::ideamaker) && gcodes_processed < 26)
                    continue;
                else if ((sliceCompany == SliceCompany::prusa || sliceCompany == SliceCompany::superslicer) && gcodes_processed < 32)
                    continue;
                else if (sliceCompany == SliceCompany::bambu && gcodes_processed < 38)
                    continue;
                else if (sliceCompany == SliceCompany::ffslicer && gcodes_processed < 14)
                    continue;

                //if (sliceCompany == SliceCompany::prusa && cmd.comment == "END gcode for filament")
                //    break;
                //if (sliceCompany == SliceCompany::prusa && cmd.comment == " filament end gcode")
                //    break;
                //if (sliceCompany == SliceCompany::bambu && cmd.comment == " filament end gcode ")
                //    break;
                //if (sliceCompany == SliceCompany::superslicer && cmd.comment == " Filament-specific end gcode ")
                //    break;
                if (sliceCompany == SliceCompany::ffslicer && cmd.comment == "end gcode")
                    break;

                if (cmd.command == "G1" && is_shell && !p_source_position_->get_current_position_ptr()->is_travel())
                {
                    // increment the number of arc commands encountered
                    num_arc_commands_++;
                    // Get the current and previous positions


                    trimesh::vec3 v3(p_source_position_->get_previous_position_ptr()->x, p_source_position_->get_previous_position_ptr()->y,
                        p_source_position_->get_previous_position_ptr()->z);
                    trimesh::vec3 v4(p_source_position_->get_current_position_ptr()->x, p_source_position_->get_current_position_ptr()->y,
                        p_source_position_->get_current_position_ptr()->z);
                    //if ((v3.x == 0.0f && v3.y == 0.0f) || (v4.x == 0.0f && v4.y == 0.0f))
                        //continue;
                    //if (sliceCompany == SliceCompany::creality && v3.x == 460.0 && v3.y == 490.0)
                    //    break;
                    SliceLine3D s;
                    s.start.x = v3.x;
                    s.start.y = v3.y;
                    s.start.z = v3.z;
                    s.end.x = v4.x;
                    s.end.y = v4.y;
                    s.end.z = v4.z;

                    box += v3;
                    box += v4;

                    m_sliceLines.emplace_back(s);
                }
                if (isLayer)
                {
                    m_sliceLayers.emplace_back(m_sliceLines);
                    m_sliceLines.clear();
                }
            }
        
            kvs.insert(std::make_pair("box_max_x", std::to_string(box.size().x)));
            kvs.insert(std::make_pair("box_max_y", std::to_string(box.size().y)));
            kvs.insert(std::make_pair("box_max_z", std::to_string(box.size().z)));
            kvs.insert(std::make_pair("layer_count", std::to_string(m_sliceLayers.size())));
            float averageThickness = box.size().z/(float)m_sliceLayers.size();
            kvs.insert(std::make_pair("averageThickness", std::to_string(averageThickness)));
        }
        gcode_file.close();

        //get filesize;
        gcode_file.open(path, std::ios_base::binary);
        gcode_file.seekg(0, std::ios_base::end);
        int nFileLen = gcode_file.tellg();
        kvs.insert(std::make_pair("fileSize", std::to_string(nFileLen)));
        gcode_file.close();
    }

    void _crealityKv(std::unordered_map<std::string, std::string>& kvs, trimesh::box3& box)
    {
        changeKey("Machine Name", "machine_name", kvs);
        changeKey("Filament used", "filament_used", kvs);
        changeKey("Material Diameter", "material_diameter", kvs);
        changeKey("TIME", "print_time", kvs);
        changeKey("Layer height", "layer_height", kvs);
        changeKey("Layer Height", "layer_height", kvs);
        changeKey("Infill Pattern", "infill_pattern", kvs);
        changeKey("Infill Sparse Density", "infill_sparse_density", kvs);
        changeKey("Support Type", "support_structure", kvs);
        changeKey("Support Pattern", "support_pattern", kvs);
        changeKey("Support Enable", "support_enable", kvs);
        changeKey("Support Angle", "support_angle", kvs);
        changeKey("Support Density", "support_infill_rate", kvs);
        changeKey("Adhesion Type", "adhesion_type", kvs);
        changeKey("Z Seam Alignment", "z_seam_type", kvs);
        changeKey("Wall Line Count", "wall_line_count", kvs);	
        changeKey("Top/Bottom Thickness", "top_bottom_thickness", kvs); 	
        changeKey("Out Wall Line Width", "wall_line_width", kvs);
        

        //changeKey("Print Speed", "speed_print", kvs);
        changeKey("Initial Layer Speed", "speed_layer_0", kvs);
        changeKey("Travel Speed", "speed_travel", kvs);
        //changeKey("Wall Speed", "speed_wall", kvs);
        changeKey("Outer Wall Speed", "speed_wall_0", kvs);
        changeKey("Inner Wall Speed", "speed_wall_x", kvs);
        changeKey("Infill Speed", "speed_infill", kvs);
        changeKey("Support Speed", "speed_support", kvs);
        changeKey("Top/Bottom Speed", "speed_topbottom", kvs);
        changeKey("Retraction Distance", "retraction_amount", kvs);
        changeKey("Retraction Speed", "retraction_speed", kvs);
        changeKey("Print Temperature", "material_print_temperature", kvs);
        changeKey("Bed Temperature", "material_bed_temperature", kvs);

        auto iter = kvs.find("Wall Thickness");
        if (iter != kvs.end())
        {
            std::string str = iter->second;
            float Wall_Thickness = atof(str.c_str());

            auto iter1 = kvs.find("Wall Line Count");
            if (iter1 != kvs.end())
            {
                std::string str1 = iter->second;
                int Wall_Line_Count = atoi(str1.c_str());

                float Wall_Line_Width = (Wall_Thickness + 0.5f) / Wall_Line_Count;
                kvs.insert(std::make_pair("wall_line_width", std::to_string(Wall_Line_Width)));
            }
        }

        //; Filament used : 20.0176m
        auto iter1 = kvs.find("Filament used");
        if (iter1 != kvs.end())
        {
            if (iter1->second.length() > 1) {
                iter1->second = iter1->second.substr(0, iter1->second.length() - 1);
            }
        }

        iter1 = kvs.find("MINX");
        if (iter1 != kvs.end())
        {
            box.min.x = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MINY");
        if (iter1 != kvs.end())
        {
            box.min.y = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MINZ");
        if (iter1 != kvs.end())
        {
            box.min.z = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MAXX");
        if (iter1 != kvs.end())
        {
            box.max.x = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MAXY");
        if (iter1 != kvs.end())
        {
            box.max.y = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MAXZ");
        if (iter1 != kvs.end())
        {
            box.max.z = atof(iter1->second.c_str());
        }
    }

    void _curaKv(std::unordered_map<std::string, std::string>& kvs, trimesh::box3& box)
    {
        auto iter = kvs.find("LAYER_COUNT");
        if (iter != kvs.end())
        {
            kvs.insert(std::make_pair("layer_count", iter->second));
        }

        //; Filament used : 20.0176m
        iter = kvs.find("Filament used");
        if (iter != kvs.end())
        {
            if (iter->second.length() > 1) {
                iter->second = iter->second.substr(0, iter->second.length() - 1);
            }
        }

        auto iter1 = kvs.find("MINX");
        if (iter1 != kvs.end())
        {
            box.min.x = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MINY");
        if (iter1 != kvs.end())
        {
            box.min.y = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MINZ");
        if (iter1 != kvs.end())
        {
            box.min.z = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MAXX");
        if (iter1 != kvs.end())
        {
            box.max.x = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MAXY");
        if (iter1 != kvs.end())
        {
            box.max.y = atof(iter1->second.c_str());
        }
        iter1 = kvs.find("MAXZ");
        if (iter1 != kvs.end())
        {
            box.max.z = atof(iter1->second.c_str());
        }

    }

    void _bambuKv(std::unordered_map<std::string, std::string>& kvs)
    {
        changeKey("total layer number", "layer_count", kvs);
        changeKey("printer_model", "machine_name", kvs);
        changeKey("model printing time", "print_time", kvs);
        changeKey("filament_diameter", "material_diameter", kvs);
        //changeKey("layer_height", "layer_height", kvs);
        changeKey("line_width", "wall_line_width", kvs);
        changeKey("sparse_infill_pattern", "infill_pattern", kvs);
        changeKey("sparse_infill_density", "infill_sparse_density", kvs);
        changeKey("support_on_build_plate_only", "support_structure", kvs);
        changeKey("enable_support", "support_enable", kvs);
        changeKey("support_threshold_angle", "support_angle", kvs);
        changeKey("outer_wall_speed", "speed_wall_0", kvs);
        changeKey("inner_wall_speed", "speed_wall_x", kvs);
        changeKey("initial_layer_speed", "speed_layer_0", kvs);
        changeKey("travel_speed", "speed_travel", kvs);
        changeKey("support_speed", "speed_support", kvs);
        changeKey("internal_solid_infill_speed", "speed_infill", kvs);
        changeKey("top_surface_speed", "speed_topbottom", kvs);
        changeKey("retraction_length", "retraction_amount", kvs);
        changeKey("retraction_speed", "retraction_speed", kvs);
        changeKey("nozzle_temperature", "material_print_temperature", kvs);
        
        
        //4h 0m 41s -> 4*3600 + 0 + 41
        auto iter = kvs.find("print_time");
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, ' ', _kvs);
            if (_kvs.size() > 2) {
                if (_kvs[0].length()>1 && _kvs[1].length() > 1 && _kvs[2].length() > 1)
                {
                    int strH = atoi(_kvs[0].substr(0, _kvs[0].length() - 1).c_str());
                    int strM = atoi(_kvs[1].substr(0, _kvs[1].length() - 1).c_str());
                    int strS = atoi(_kvs[2].substr(0, _kvs[2].length() - 1).c_str());
                    int time = strH * 3600 + strM * 60 + strS;
                    iter->second = std::to_string(time);
                }
            }
        }

        //; filament_diameter = 1.75,1.75
        auto iter1 = kvs.find("material_diameter");
        if (iter1 != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter1->second, ',', _kvs);
            if (_kvs.size() > 1) {
                iter1->second = _kvs[0];

                for (int i = 1; i < _kvs.size(); i++)
                {
                    kvs.insert(std::make_pair("material_diameter" + std::to_string(i), _kvs[i]));
                }
            }
        }

        //; nozzle_temperature = 220,220
        iter1 = kvs.find("material_print_temperature");
        if (iter1 != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter1->second, ',', _kvs);
            if (_kvs.size() > 1) {
                iter1->second = _kvs[0];

                for (int i = 1; i < _kvs.size(); i++)
                {
                    kvs.insert(std::make_pair("material_print_temperature" + std::to_string(i), _kvs[i]));
                }
            }
        }

        auto iter2 = kvs.find("support_enable");
        if (iter2 != kvs.end())
        {
            int support = atoi(iter2->second.c_str());
            if (support != 0)
            {
                iter2->second = "true";
            }
            else
                iter2->second = "false";
        }

        //; support_on_build_plate_only = 0
        auto iter3 = kvs.find("support_structure");
        if (iter3 != kvs.end())
        {
            int support = atoi(iter3->second.c_str());
            if (support != 0)
            {
                iter3->second = "everywhere";
            }
            else
                iter3->second = "buildplate";
        }

        //;  = 0
        auto iter4 = kvs.find("skirt_loops");
        if (iter4 != kvs.end())
        {
            int skirt = atoi(iter4->second.c_str());
            if (skirt != 0)
            {
                iter4->second = "brim";
            }
            else
                iter4->second = "skirt";
        }
        changeKey("skirt_loops", "adhesion_type", kvs);
       
        //; layer num/total_layer_count: 45/500
        auto iter5 = kvs.find("layer num/total_layer_count");
        if (iter5 != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter5->second, '/', _kvs);
            if (_kvs.size() > 1) {
                iter5->second = _kvs[1];
            }
        }
        changeKey("layer num/total_layer_count", "wall_line_count", kvs);

        //; cool_plate_temp = 35,35
        iter5 = kvs.find("cool_plate_temp");
        if (iter5 != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter5->second, ',', _kvs);
            if (_kvs.size() > 0) {
                iter5->second = _kvs[0];
            }
        }
        changeKey("cool_plate_temp", "material_bed_temperature", kvs);    

        auto iter6 = kvs.find("top_shell_thickness");
        if (iter6 != kvs.end())
        {
            float top_thickness = atof(iter6->second.c_str());
            float bottom_thickness = 0.0f;
            iter6 = kvs.find("bottom_shell_thickness");
            if (iter6 != kvs.end())
            {
                bottom_thickness = atof(iter6->second.c_str());
            }
            float top_bottom_thickness = std::max(top_thickness, bottom_thickness);
            kvs.insert(std::make_pair("top_bottom_thickness",std::to_string(top_bottom_thickness)));        
        }
           
    }

    void _ffslicerKv(std::unordered_map<std::string, std::string>& kvs)
    {
        changeKey("layer_count", "layer_count", kvs);
        changeKey("machine_type", "machine_name", kvs);
        changeKey("filament_diameter0", "material_diameter", kvs);
        //changeKey("layer_height", "layer_height", kvs);
        changeKey("fill_pattern", "infill_pattern", kvs);
        changeKey("fill_density", "infill_sparse_density", kvs);
        changeKey("perimeter_shells", "wall_line_count", kvs);
        changeKey("base_print_speed", "speed_print", kvs);
        
        changeKey("travel_speed", "speed_travel", kvs);
        changeKey("platform_temperature", "material_bed_temperature", kvs);
        
        auto iter = kvs.find("top_solid_layers");
        if (iter != kvs.end())
        {
            float top_thickness = atof(iter->second.c_str());
            float bottom_thickness = 0.0f;
            iter = kvs.find("bottom_solid_layers");
            if (iter != kvs.end())
            {
                bottom_thickness = atof(iter->second.c_str());
            }
            float top_bottom_thickness = std::max(top_thickness, bottom_thickness);
            kvs.insert(std::make_pair("top_bottom_thickness", std::to_string(top_bottom_thickness)));
        }

        iter = kvs.find("right_extruder_temperature");
        if (iter != kvs.end())
        {
            changeKey("right_extruder_temperature", "material_print_temperature", kvs);
        }
        else
        {
            iter = kvs.find("left_extruder_temperature");
            if (iter != kvs.end())
            {
                changeKey("left_extruder_temperature", "material_print_temperature", kvs);
            }
        }
    }

    void _ideamakerKv(std::unordered_map<std::string, std::string>& kvs, trimesh::box3& box)
    {
        float material1 = 0.0f;
        float material2 = 0.0f;
        auto iter = kvs.find("Material#1 Used");
        if (iter != kvs.end())
        {
            material1 = atof(iter->second.c_str());
        }
        iter = kvs.find("Material#2 Used");
        if (iter != kvs.end())
        {
            material2 = atof(iter->second.c_str());
        }
        kvs.insert(std::make_pair("filament_used", std::to_string(material1 + material2)));

        //; Filament Diameter #1: 1.750
        iter = kvs.find("Filament Diameter #1");
        if (iter != kvs.end())
        {
            changeKey("Filament Diameter #1", "material_diameter", kvs);
        }
        else
        {
            iter = kvs.find("Filament Diameter #2");
            if (iter != kvs.end())
            {
                changeKey("Filament Diameter #2", "material_diameter", kvs);
            }
        }

        changeKey("Print Time", "print_time", kvs);
        
        iter = kvs.find("Bounding");
        if (iter != kvs.end())
        {
            std::vector<std::string> minMax;
            Stringsplit(iter->second, ' ', minMax);
            if(minMax.size() == 6)
            {
                box.min.x = atof(minMax.at(0).c_str());
                box.min.y = atof(minMax.at(1).c_str());
                box.min.z = atof(minMax.at(2).c_str());
                box.max.x = atof(minMax.at(3).c_str());
                box.max.y = atof(minMax.at(4).c_str());
                box.max.z = atof(minMax.at(5).c_str());
            }
        }

    }

    void _prusaKv(std::unordered_map<std::string, std::string>& kvs)
    {
        changeKey("printer_model", "machine_name", kvs);
        changeKey("filament used [mm]", "filament_used", kvs);
        changeKey("filament_diameter", "material_diameter", kvs);
        
        changeKey("filament_diameter", "material_diameter", kvs);

        //4h 0m 41s -> 4*3600 + 0 + 41
        auto iter = kvs.find("estimated printing time (normal mode)");
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, ' ', _kvs);
            if (_kvs.size() > 2) {
                if (_kvs[0].length() > 1 && _kvs[1].length() > 1 && _kvs[2].length() > 1)
                {
                    int strH = atoi(_kvs[0].substr(0, _kvs[0].length() - 1).c_str());
                    int strM = atoi(_kvs[1].substr(0, _kvs[1].length() - 1).c_str());
                    int strS = atoi(_kvs[2].substr(0, _kvs[2].length() - 1).c_str());
                    int time = strH * 3600 + strM * 60 + strS;
                    iter->second = std::to_string(time);
                }
            }
        }
        changeKey("estimated printing time (normal mode)", "print_time", kvs);
        //changeKey("layer_height", "layer_height", kvs);
        changeKey("external perimeters extrusion width", "wall_line_width", kvs);
        changeKey("fill_pattern", "infill_pattern", kvs);
        changeKey("fill_density", "infill_sparse_density", kvs);
        changeKey("support_material_style", "support_pattern", kvs);

        //; support_on_build_plate_only = 0
        auto iter3 = kvs.find("support_material_buildplate_only");
        if (iter3 != kvs.end())
        {
            int support = atoi(iter3->second.c_str());
            if (support != 0)
            {
                iter3->second = "everywhere";
            }
            else
                iter3->second = "buildplate";
            changeKey("support_material_buildplate_only", "support_structure", kvs);
        }
        
        changeKey("support_material_threshold", "support_angle", kvs);
        


        //; support_on_build_plate_only = 0
        iter3 = kvs.find("brim_type");
        if (iter3 != kvs.end())
        {
            if (iter3->second != "no_brim")
                iter3->second = "brim";
            else
                iter3->second = "none";

            changeKey("brim_type", "adhesion_type", kvs);
        }
            
        changeKey("perimeter_speed", "speed_print", kvs);
        changeKey("first_layer_speed", "speed_layer_0", kvs);
        changeKey("travel_speed", "speed_travel", kvs);
        changeKey("external_perimeter_speed", "speed_wall_0", kvs);
        changeKey("infill_speed", "speed_infill", kvs);
        changeKey("support_material_speed", "speed_support", kvs);
        changeKey("top_solid_infill_speed", "speed_topbottom", kvs);
        changeKey("retract_length", "retraction_amount", kvs);
        changeKey("retract_speed", "retraction_speed", kvs);
        
        changeKey("temperature", "material_print_temperature", kvs);
        changeKey("bed_temperature", "material_bed_temperature", kvs);
    }

    void _simplifyKv(std::unordered_map<std::string, std::string>& kvs)
    {     
        //Filament length : 172129.4 mm(172.13 m)
        auto iter = kvs.find("Filament length");
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, 'mm', _kvs);
            if (_kvs.size()>0)
            {
                float len = std::atoi(_kvs[0].c_str())/1000.0f;
                iter->second = std::to_string(len);
                changeKey("Filament length", "filament_used", kvs);            
            }
        }

        //filamentDiameters,1.75|1.75|1.75|1.75|1.75|1.75
        iter = kvs.find("filamentDiameters");
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, '|', _kvs);
            if (_kvs.size() > 0)
            {
                iter->second = _kvs[0];    
            }
            changeKey("filamentDiameters", "material_diameter", kvs);
        }

        //Build time : 29 hours 56 minutes
        iter = kvs.find("Build time");
        if (iter != kvs.end())
        {
            int pos = iter->second.find("hours");
            std::string h = "";
            std::string m = "";
            if (pos >=1 )
            {
                h = iter->second.substr(0, pos - 1);
                std::string min = iter->second.substr(pos + 5, iter->second.length());
                m = min.substr(0, 3);
            }
            else
            {
                pos = iter->second.find("minutes");
                if (pos >= 1 )
                {
                    m = iter->second.substr(0, pos - 1);
                }
            }
            float s = std::atoi(h.c_str()) * 3600 + std::atoi(m.c_str()) * 60;
            iter->second = std::to_string(s);
            changeKey("Build time", "print_time", kvs);
        }

        changeKey("layerHeight", "layer_height", kvs);
        changeKey("extruderWidth", "wall_line_width", kvs);
        
        changeKey("internalInfillPattern", "infill_pattern", kvs);
        changeKey("externalInfillPattern", "infill_pattern", kvs);
        changeKey("infillPercentage", "infill_sparse_density", kvs);
        
        changeKey("generateSupport", "support_enable", kvs);
        iter = kvs.find("support_enable");
        if (iter != kvs.end())
        {
            int support = atoi(iter->second.c_str());
            if (support != 0)
            {
                iter->second = "true";
            }
            else
                iter->second = "false";
        }

        changeKey("supportType", "support_structure", kvs);
        //; support_on_build_plate_only = 0
        auto iter3 = kvs.find("support_structure");
        if (iter3 != kvs.end())
        {
            int support = atoi(iter3->second.c_str());
            if (support == 0)
            {
                iter3->second = "everywhere";
            }
            else
                iter3->second = "buildplate";
        }

        changeKey("supportAngles", "support_angle", kvs);
        changeKey("supportInfillPercentage", "support_infill_rate", kvs);
        iter3 = kvs.find("support_infill_rate");
        if (iter3 != kvs.end())
        {
            float density = atoi(iter3->second.c_str()) / 100.0f;
            iter3->second = std::to_string(density);
        }

        //Adhesion Type
        //"lace": "Lace",
        //"skirt": "skirt",
        //    "brim" : "brim",
        //    "autobrim" : "autobrim",
       //     "raft" : "raft",
      //      "none" : "none"
        //useRaft, 1
       //useSkirt, 1
       //usePrimePillar, 0
       //useOozeShield, 0


        iter3 = kvs.find("useRaft");
        if (iter3->second == "1")
            kvs.insert(std::make_pair("adhesion_type", "raft"));
        else
        {
            iter3 = kvs.find("useSkirt");
            if (iter3 != kvs.end())
            {
                if (iter3->second == "1")
                    kvs.insert(std::make_pair("adhesion_type", "skirt"));
                else
                    kvs.insert(std::make_pair("adhesion_type", "none"));
            }
            else
                kvs.insert(std::make_pair("adhesion_type", "none"));
        }
      
        iter3 = kvs.find("startPointOption");
        if (iter3->second == "2")
        {   
            kvs.insert(std::make_pair("z_seam_type", "true"));
        }
        else
            kvs.insert(std::make_pair("z_seam_type", "false"));
        
        changeKey("perimeterOutlines", "wall_line_count", kvs);
        
        iter3 = kvs.find("topSolidLayers");
        if (iter3 != kvs.end())
        {
            float num = std::atof(iter3->second.c_str());
            iter3 = kvs.find("layer_height");
            if (iter3 != kvs.end())
            {
                float thickness = num * std::atof(iter3->second.c_str());
                kvs.insert(std::make_pair("top_bottom_thickness", std::to_string(thickness)));
            }
        }

        changeKey("defaultSpeed", "speed_print", kvs);
        float defaultSpeed = 0.0f;
        iter3 = kvs.find("speed_print");
        if (iter3 != kvs.end())
        {
            defaultSpeed = std::atof(iter3->second.c_str()) / 60.0f;
            iter3->second = std::to_string(defaultSpeed);
        }
        
        //
        changeKey("firstLayerUnderspeed", "speed_layer_0", kvs);
        iter3 = kvs.find("speed_layer_0");
        if (iter3 != kvs.end())
        {
            float s = std::atof(iter3->second.c_str());
            iter3->second = std::to_string(s * defaultSpeed);
        }

        //rapidXYspeed, 4800
        changeKey("rapidXYspeed", "speed_travel", kvs);
        iter3 = kvs.find("speed_travel");
        if (iter3 != kvs.end())
        {
            float defaultSpeed = std::atof(iter3->second.c_str()) / 60.0f;
            iter3->second = std::to_string(defaultSpeed);
        }

        //Outer Wall Speed  outlineUnderspeed
        changeKey("outlineUnderspeed", "speed_wall_0", kvs);
        iter3 = kvs.find("speed_wall_0");
        if (iter3 != kvs.end())
        {
            float s = std::atof(iter3->second.c_str());
            iter3->second = std::to_string(s * defaultSpeed);
        }
       
        changeKey("solidInfillUnderspeed", "speed_infill", kvs);
        iter3 = kvs.find("speed_infill");
        if (iter3 != kvs.end())
        {
            float s = std::atof(iter3->second.c_str());
            iter3->second = std::to_string(s * defaultSpeed);
        }

        //Support Speed
        changeKey("solidInfillUnderspeed", "speed_support", kvs);
        iter3 = kvs.find("speed_support");
        if (iter3 != kvs.end())
        {
            float s = std::atof(iter3->second.c_str());
            iter3->second = std::to_string(s * defaultSpeed);
        }

        //
        changeKey("toolChangeRetractionDistance", "retraction_amount", kvs);
        changeKey("toolChangeRetractionSpeed", "retraction_speed", kvs);
        iter3 = kvs.find("retraction_speed");
        if (iter3 != kvs.end())
        {
            float defaultSpeed = std::atof(iter3->second.c_str()) / 60.0f;
            iter3->second = std::to_string(defaultSpeed);
        }

        //temperatureSetpointTemperatures, 190, 200, 45
        //temperatureNumber,0,1,0
        iter = kvs.find("temperatureNumber");
        auto iter1 = kvs.find("temperatureSetpointTemperatures");
        auto iter2 = kvs.find("temperatureHeatedBed"); 
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, ',', _kvs);
            
            std::vector<std::string> _kvs1;
            Stringsplit(iter1->second, ',', _kvs1);

            std::vector<std::string> _kvs2;
            Stringsplit(iter2->second, ',', _kvs2);

            if (_kvs.size() == _kvs1.size() && _kvs.size() == _kvs2.size())
            {
                for (int i = 0; i < _kvs.size(); i++)
                {
                    int num = std::atoi(_kvs[i].c_str());
                    int isBed = std::atoi(_kvs2[i].c_str());
                    if (num == 0 && isBed == 0)
                    {
                        kvs.insert(std::make_pair("material_print_temperature",_kvs1[i]));
                    }
                    else if (num > 0 && isBed == 0)
                    {
                        kvs.insert(std::make_pair("material_print_temperature" + std::to_string(num), _kvs1[i]));
                    }
                    else if (isBed)
                    {
                        kvs.insert(std::make_pair("material_bed_temperature", _kvs1[i]));
                    }
                }
            }
        }     
    }

    void _superslicerKv(std::unordered_map<std::string, std::string>& kvs)
    {
        changeKey("printer_model", "machine_name", kvs);
        changeKey("filament used [mm]", "filament_used", kvs);
        changeKey("filament_diameter", "material_diameter", kvs);

        //4h 0m 41s -> 4*3600 + 0 + 41
        auto iter = kvs.find("estimated printing time (normal mode)");
        if (iter != kvs.end())
        {
            std::vector<std::string> _kvs;
            Stringsplit(iter->second, ' ', _kvs);
            if (_kvs.size() > 2) {
                if (_kvs[0].length() > 1 && _kvs[1].length() > 1 && _kvs[2].length() > 1)
                {
                    int strH = atoi(_kvs[0].substr(0, _kvs[0].length() - 1).c_str());
                    int strM = atoi(_kvs[1].substr(0, _kvs[1].length() - 1).c_str());
                    int strS = atoi(_kvs[2].substr(0, _kvs[2].length() - 1).c_str());
                    int time = strH * 3600 + strM * 60 + strS;
                    iter->second = std::to_string(time);
                }
            }
        }
        changeKey("estimated printing time (normal mode)", "print_time", kvs);
        //changeKey("layer_height", "layer_height", kvs);
        changeKey("external perimeters extrusion width", "wall_line_width", kvs);
        changeKey("fill_pattern", "infill_pattern", kvs);
        changeKey("fill_density", "infill_sparse_density", kvs);
        changeKey("support_material_style", "support_pattern", kvs);

        //; support_on_build_plate_only = 0
        auto iter3 = kvs.find("support_material_buildplate_only");
        if (iter3 != kvs.end())
        {
            int support = atoi(iter3->second.c_str());
            if (support != 0)
            {
                iter3->second = "everywhere";
            }
            else
                iter3->second = "buildplate";
            changeKey("support_material_buildplate_only", "support_structure", kvs);
        }

        changeKey("support_material_threshold", "support_angle", kvs);



        //; support_on_build_plate_only = 0
        iter3 = kvs.find("brim_type");
        if (iter3 != kvs.end())
        {
            if (iter3->second != "no_brim")
                iter3->second = "brim";
            else
                iter3->second = "none";

            changeKey("brim_type", "adhesion_type", kvs);
        }

        changeKey("perimeter_speed", "speed_print", kvs);
        changeKey("first_layer_speed", "speed_layer_0", kvs);
        changeKey("travel_speed", "speed_travel", kvs);
        changeKey("external_perimeter_speed", "speed_wall_0", kvs);
        changeKey("infill_speed", "speed_infill", kvs);
        changeKey("support_material_speed", "speed_support", kvs);
        changeKey("top_solid_infill_speed", "speed_topbottom", kvs);
        changeKey("retract_length", "retraction_amount", kvs);
        changeKey("retract_speed", "retraction_speed", kvs);

        changeKey("temperature", "material_print_temperature", kvs);
        changeKey("bed_temperature", "material_bed_temperature", kvs);
    }

    void _paraseKvs(const SliceCompany& sliceCompany, trimesh::box3& box,std::unordered_map<std::string, std::string>& kvs)
    {
        switch (sliceCompany)
        {
        case SliceCompany::creality:
            _crealityKv(kvs, box);
            break;
        case SliceCompany::cura:
            _curaKv(kvs, box);
            break;
        case SliceCompany::bambu:
            _bambuKv(kvs);
            break;
        case SliceCompany::ffslicer:
            _ffslicerKv(kvs);
            break;
        case SliceCompany::ideamaker:
            _ideamakerKv(kvs, box);
            break;
        case SliceCompany::prusa:
            _prusaKv(kvs);
            break;
        case SliceCompany::simplify:
            _simplifyKv(kvs);
            break;
        case SliceCompany::superslicer:
            _superslicerKv(kvs);
            break;
        default:
            break;
        }
      
        auto iter = kvs.find("layer_height");
        if (iter == kvs.end())
        {
            auto iter1 = kvs.find("averageThickness");
            if (iter != kvs.end())
            {
                kvs.insert(std::make_pair("layer_height", iter1->second));
            }
        }


    }

    void _removeOthersKvs(std::unordered_map<std::string, std::string>& kvs)
    {
        std::vector<std::string> _kvs;
        _kvs.push_back("box_max_y");
        _kvs.push_back("box_max_z");
        _kvs.push_back("fileSize");
        _kvs.push_back("box_max_x");
        _kvs.push_back("box_max_y");
        _kvs.push_back("box_max_z");
        _kvs.push_back("layer_count");
        _kvs.push_back("machine_name");
        _kvs.push_back("filament_used");
        _kvs.push_back("material_diameter");
        _kvs.push_back("print_time");
        _kvs.push_back("layer_height");
        _kvs.push_back("wall_line_width");
        _kvs.push_back("infill_pattern");
        _kvs.push_back("infill_sparse_density");
        _kvs.push_back("support_enable");
        _kvs.push_back("support_structure");
        _kvs.push_back("support_pattern");
        _kvs.push_back("support_angle");
        _kvs.push_back("support_infill_rate");
        _kvs.push_back("adhesion_type");
        _kvs.push_back("z_seam_type");
        _kvs.push_back("wall_line_count");
        _kvs.push_back("top_bottom_thickness");
        _kvs.push_back("speed_print");
        _kvs.push_back("speed_layer_0");
        _kvs.push_back("speed_travel");
        _kvs.push_back("speed_wall");
        _kvs.push_back("speed_wall_0");
        _kvs.push_back("speed_wall_x");
        _kvs.push_back("speed_infill");
        _kvs.push_back("speed_support");
        _kvs.push_back("speed_topbottom");
        _kvs.push_back("retraction_amount");
        _kvs.push_back("retraction_speed");
        _kvs.push_back("material_print_temperature");
        _kvs.push_back("material_bed_temperature");

        std::unordered_map<std::string, std::string> Newkvs;
        for (const auto& key : _kvs)
        {
            auto iter = kvs.find(key);
            if (iter != kvs.end())
            {
                Newkvs.insert(std::make_pair(iter->first, iter->second));
            }
        }

        kvs.clear();
        kvs.swap(Newkvs);
    }

    void paraseGcode(const std::string& gCodeFile, std::vector<std::vector<SliceLine3D>>& m_sliceLayers, trimesh::box3& box, std::unordered_map<std::string, std::string>& kvs)
    {
        SliceCompany sliceCompany = SliceCompany::none;

        //获取原始数据
        _paraseGcode(sliceCompany,gCodeFile,box,m_sliceLayers,kvs);

        //解析成通用参数
        _paraseKvs(sliceCompany, box, kvs);

        //过滤其他参数
        _removeOthersKvs(kvs);
    }
}