#include "gcodedata.h"
#include "gcode/sliceresult.h"
#include <regex>

namespace cxsw
{
	bool gcodeGenerate(const std::string& fileName, gcode::GCodeStruct& _gcodeStruct, ccglobal::Tracer* tracer)
	{
		gcode::SliceResult* result=new gcode::SliceResult();
		result->load(fileName, tracer);
		_gcodeStruct.buildFromResult(result, tracer);
		return true;
	}

	GCODE_API bool _SaveGCode(const std::string& fileName, const std::string& previewImageString, const std::vector<std::string>& layerString, const std::string& prefixString, const std::string& tailCode)
	{
		std::ofstream out(fileName);

		if (!out.is_open())
		{
			out.close();
			return false;
		}

		///添加预览图信息
		if (!previewImageString.empty())
		{
			out.write(previewImageString.c_str(), previewImageString.size());
			out.write("\r\n", 2); //in << endl;
		}

		//添加打印时间等信息
		if (!prefixString.empty())
		{
			out.write(prefixString.c_str(), prefixString.size());
			out.write("\n", 1);
		}

		for (const std::string& layerLine : layerString)
		{
			out.write(layerLine.c_str(), layerLine.size());
			out.write("\n", 1);
		}

		if (!tailCode.empty())
		{
			out.write(tailCode.c_str(), tailCode.size());
			out.write("\n", 1);
		}
		out.close();
		return true;
	}

	bool  regex_match(std::string& gcodeStr, std::string key, std::smatch& sm)
	{
		std::string temp1 = ".*" + key + ":([0-9]{0,8}).*";
		std::string temp2 = ".*" + key + ":([-]{0,1}[0-9]{0,8}\\.[0-9]{0,8}).*";
		if (std::regex_match(gcodeStr, sm, std::regex(temp1.c_str())) ||
			std::regex_match(gcodeStr, sm, std::regex(temp2.c_str())))
		{
			return true;
		}
		return false;
	}

	bool  regex_match_float(std::string& gcodeStr, std::string key, std::smatch& sm)
	{
		std::string temp1 = ".*" + key + ":([-]{0,1}[0-9]{0,8}\\.[0-9]{0,8}).";
		std::string temp2 = ".*" + key + ":([-]{0,1}[0-9]{0,8}\\.[0-9]{0,8}).*";
		if (std::regex_match(gcodeStr, sm, std::regex(temp1.c_str())) ||
			std::regex_match(gcodeStr, sm, std::regex(temp2.c_str())))
		{
			return true;
		}
		return false;
	}

	bool  regex_match_time(std::string& gcodeStr, std::smatch& sm, gcode::GCodeParseInfo& parseInfo)
	{
		if (std::regex_match(gcodeStr, sm, std::regex(".*OuterWall Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.OuterWall = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*InnerWall Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.InnerWall = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*Skin Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.Skin = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*Support Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.Support = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*SkirtBrim Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.SkirtBrim = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*Infill Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.Infill = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*InfillSupport Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.SupportInfill = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*Combing Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.MoveCombing = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*Retraction Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.MoveRetraction = tmp;
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*PrimeTower Time:([0-9]{0,8}).*")))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.timeParts.PrimeTower = tmp;
		}

		return true;
	}

	void parseGCodeInfo(gcode::SliceResult* result, gcode::GCodeParseInfo& parseInfo)
	{
		std::string gcodeStr = result->prefixCode();

		std::replace(gcodeStr.begin(), gcodeStr.end(), '\n', ' ');
		std::replace(gcodeStr.begin(), gcodeStr.end(), '\r', ' ');
		std::smatch sm;

		if (std::regex_match(gcodeStr, sm, std::regex(".*TIME:([0-9]{0,8}).*"))) ////get print time
		{
			std::string tStr = sm[1];
			int tmp = atoi(tStr.c_str());
			parseInfo.printTime = tmp;

			regex_match_time(gcodeStr, sm, parseInfo);
		}
		if (std::regex_match(gcodeStr, sm, std::regex(".*Filament used:([0-9]{0,8}\\.[0-9]{0,8}).*"))) ////get print time
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.materialLenth = tmp;
		}

		if (regex_match(gcodeStr, "machine belt offset", sm))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.beltOffset = tmp;
		}

		if (regex_match(gcodeStr, "machine belt offset Y", sm))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.beltOffsetY = tmp;
		}

		bool hasMachineSpaceBox = false;
		if (regex_match(gcodeStr, "Machine Height", sm))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.machine_height = tmp;
		}
		else
			hasMachineSpaceBox = true;

		if (regex_match(gcodeStr, "Machine Width", sm))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.machine_width = tmp;
		}
		else
			hasMachineSpaceBox = true;

		if (regex_match(gcodeStr, "Machine Depth", sm))
		{
			std::string tStr = sm[1];
			float tmp = atof(tStr.c_str());
			parseInfo.machine_depth = tmp;
		}
		else
			hasMachineSpaceBox = true;

		if (hasMachineSpaceBox)
		{//获取模型尺寸信息
			if (regex_match(gcodeStr, "MAXX", sm))
			{
				std::string tStr = sm[1];
				parseInfo.machine_width = atof(tStr.c_str()) + 20; //gap
			}

			if (regex_match(gcodeStr, "MAXY", sm))
			{
				std::string tStr = sm[1];
				parseInfo.machine_depth = atof(tStr.c_str()) + 20; //gap
			}

			if (regex_match(gcodeStr, "MAXZ", sm))
			{
				std::string tStr = sm[1];
				parseInfo.machine_height = atof(tStr.c_str()) + 20; //gap
			}
		}

		if (gcodeStr.find("M83") != std::string::npos)
		{
			parseInfo.relativeExtrude = true;
		}

		//float material_diameter = 1.75;
		//float material_density = 1.24;
		if (regex_match(gcodeStr, "Material Diameter", sm))
		{
			std::string tStr = sm[1];
			parseInfo.material_diameter = atof(tStr.c_str()); //gap
		}
		if (regex_match(gcodeStr, "Material Density", sm))
		{
			std::string tStr = sm[1];
			parseInfo.material_density = atof(tStr.c_str()); //gap
		}
		//单位面积密度
		parseInfo.materialDensity = PI * (parseInfo.material_diameter * 0.5) * (parseInfo.material_diameter * 0.5) * parseInfo.material_density;

		float filament_cost = 0.0;
		if (regex_match(gcodeStr, "Filament Cost", sm))
		{
			std::string tStr = sm[1];
			filament_cost = atof(tStr.c_str()); //gap
		}
		float filament_weight = 0.0;
		if (regex_match(gcodeStr, "Filament Weight", sm))
		{
			std::string tStr = sm[1];
			filament_weight = atof(tStr.c_str()); //gap
		}

		float filament_length = filament_weight / parseInfo.materialDensity;
		parseInfo.unitPrice = filament_cost / filament_length;

		parseInfo.lineWidth = 0.4;
		if (regex_match_float(gcodeStr, "Out Wall Line Width", sm))
		{
			std::string tStr = sm[1];
			parseInfo.lineWidth = atof(tStr.c_str()); //gap
		}

		parseInfo.exportFormat = "jpg";
		int ipos = gcodeStr.find("Preview Img Type");
		if (ipos != std::string::npos)
		{
			parseInfo.exportFormat = gcodeStr.substr(ipos + 17, 3);
		}


		parseInfo.layerHeight = 0.1;
		if (regex_match_float(gcodeStr, "Layer Height", sm))
		{
			std::string tStr = sm[1];
			parseInfo.layerHeight = atof(tStr.c_str()); //gap
			//兼容老的
			if (parseInfo.layerHeight > 50)
				parseInfo.layerHeight = parseInfo.layerHeight / 1000.0f;
		}
		parseInfo.screenSize = "Sermoon D3";
		if (gcodeStr.find("Screen Size:CR-200B Pro") != std::string::npos)
		{
			parseInfo.screenSize = "CR - 200B Pro";
		}
		else if (gcodeStr.find("Screen Size:CR-10 Inspire") != std::string::npos)
		{
			parseInfo.screenSize = "CR-10 Inspire";
		}

		const std::string& preStr = result->prefixCode();
		parseInfo.spiralMode = preStr.find(";Vase Model:true");
		if (preStr.find(";machine is belt:true") != std::string::npos)
			parseInfo.beltType = 1;
		if (preStr.find("Crealitybelt") != std::string::npos)
			parseInfo.beltType = 2;
	}

}
