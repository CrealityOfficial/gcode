#include "gcodedata.h"
#include "gcode/sliceresult.h"


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

}
