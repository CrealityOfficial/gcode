#ifndef _GCODE_NULLSPACE_SLICEMODELBUILDER_1590033412412_H
#define _GCODE_NULLSPACE_SLICEMODELBUILDER_1590033412412_H
#include "gcode/interface.h"
#include <float.h>
#include <vector>
#include <map>
#include "gcode/sliceline.h"
#include "gcode/sliceresult.h"
#include "gcode/define.h"
#include "ccglobal/tracer.h"
#include "trimesh2/Box.h"

namespace gcode
{
	struct GCodeParseInfo;
	struct GCodeStructBaseInfo
	{
		std::vector<int> layerNumbers;
		std::vector<int> steps;
		int totalSteps;
		int layers;
		float minTimeOfLayer;   //单层最短时间
		float maxTimeOfLayer;   //单层最长时间

		float minFlowOfStep;  //单步最小流量，非零
		float maxFlowOfStep;  //单步最大流量

		float minLineWidth;
		float maxLineWidth;

		float minLayerHeight;
		float maxLayerHeight;

		float minTemperature;
		float maxTemperature;

		trimesh::box3 gCodeBox;
		int nNozzle;
		float speedMin;
		float speedMax;

		GCodeStructBaseInfo()
			: speedMin(FLT_MAX)
			, speedMax(FLT_MIN)
			, layers(0)
			, nNozzle(0)
			, totalSteps(0)
			, minTimeOfLayer(FLT_MAX)
			, maxTimeOfLayer(FLT_MIN)
			, minFlowOfStep(FLT_MAX)
			, maxFlowOfStep(FLT_MIN)
			, minLineWidth(FLT_MAX)
			, maxLineWidth(FLT_MIN)
			, minLayerHeight(FLT_MAX)
			, maxLayerHeight(FLT_MIN)
			, minTemperature(FLT_MAX)
			, maxTemperature(FLT_MIN)
		
		{
		}
	};

	struct GcodeTemperature
	{
		float bedTemperature{ 0.0f }; //平台温度
		float temperature{ 0.0f };  //喷嘴温度
		float camberTemperature{ 0.0f };  //腔体温度
	};

	struct GcodeFan
	{
		float fanSpeed{ 0.0f }; //风扇速度
		float camberSpeed{ 0.0f };  //腔体风扇速度
		float fanSpeed_1{ 0.0f };  //子风扇
	};

	struct GcodeLayerInfo
	{
		float layerHight{ 0.0f }; //层高
		float width{ 0.0f };  //线宽
		float flow{ 0.0f }; //体积流量
	};

	struct GCodeMove
	{
		int start;
		float speed;
		SliceLineType type;
		float e;  //流量
		int extruder;
	};

	class GCODE_API GCodeStruct
	{
	public:
		GCodeStruct();
		~GCodeStruct();

		//void buildFromResult(SliceResultPointer result, const GCodeParseInfo& info, GCodeStructBaseInfo& baseInfo, QVector<QList<int>>& stepIndexMaps, ccglobal::Tracer* tracer = nullptr);
		void buildFromResult(gcode::SliceResult* result, ccglobal::Tracer* tracer = nullptr);
		void parseGCodeInfo(gcode::SliceResult* result);

		std::vector<trimesh::vec3> m_positions;
		std::vector<GCodeMove> m_moves;  //流量、速度..

		std::vector <GcodeTemperature> m_temperatures;//温度设置值
		std::vector<int> m_temperatureIndex;//温度步进索引
		std::vector <GcodeFan> m_fans;//风扇设置值
		std::vector<int> m_fanIndex;//风扇步进索引
		std::vector <GcodeLayerInfo> m_gcodeLayerInfos;  //层高、线宽设置值
		std::vector<int> m_layerInfoIndex;  //层高、线宽 步进索引

		std::map<int,float> m_layerTimes;  //每层时间
		//std::map<int, float> m_layerTimeLogs;  //每层时间对数

		std::vector<int> m_zSeams;
		std::vector<int> m_retractions;
	private:
		void processLayer(const std::string& layerCode, int layer, std::list<int>& stepIndexMap);
		void processStep(const std::string& stepCode, int nIndex, std::list<int>& stepIndexMap);
		void processG01(const std::string& G01Str, int nIndex, std::list<int>& stepIndexMap,bool isG2G3 =false);
		void processG23(const std::string& G23Str, int nIndex, std::list<int>& stepIndexMap);
		void processSpeed(float speed);

		void processPrefixCode(const std::string& stepCod);
		void checkoutFan(const std::string& stepCod);
		void checkoutTemperature(const std::string& stepCode);
		void checkoutLayerInfo(const std::string& stepCode,int layer);
		void checkoutLayerHeight(const std::vector<std::string>& layerLines);
	protected:
		SliceLineType  tempCurrentType;
		int tempNozzleIndex;
		float tempCurrentE;
		float tempCurrentTime{ 0.0f };
		float tempCurrentZ{ 0.0f };
		float belowZ{ 0.0f };
		trimesh::vec3 tempCurrentPos;
		float tempSpeed;
		bool layerNumberParseSuccess;

		GCodeParseInfo parseInfo;
		GCodeStructBaseInfo tempBaseInfo;

		ccglobal::Tracer* m_tracer;
	};
}
#endif // _GCODE_NULLSPACE_SLICEMODELBUILDER_1590033412412_H
