#include "slicemodelbuilder.h"
#include "gcode/sliceresult.h"
//#include "slice/sliceattain.h"
#include "trimesh2/XForm.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/trimesh/algrithm3d.h"
#include <math.h>
#include "stringutil/util.h"
#include <regex>
#include "thumbnail/thumbnail.h"
#include "gcodedata.h"


namespace gcode
{
    inline SliceLineType GetLineType(const std::string& strLineType)
    {
        if (strLineType.find(";TYPE:WALL-OUTER") != std::string::npos)
        {
            return SliceLineType::OuterWall;
        }
        else if (strLineType.find(";TYPE:WALL-INNER") != std::string::npos)
        {
            return SliceLineType::InnerWall;
        }
        else if (strLineType.find(";TYPE:SKIN") != std::string::npos)
        {
            return SliceLineType::Skin;
        }
        else if (strLineType.find(";TYPE:SUPPORT-INTERFACE") != std::string::npos)
        {
            return SliceLineType::SupportInterface;
        }
        else if (strLineType.find(";TYPE:SUPPORT-INFILL") != std::string::npos)
        {
            return SliceLineType::SupportInfill;
        }
        else if (strLineType.find(";TYPE:SUPPORT") != std::string::npos)
        {
            return SliceLineType::Support;
        }
        else if (strLineType.find(";TYPE:SKIRT") != std::string::npos)
        {
            return SliceLineType::SkirtBrim;
        }
        else if (strLineType.find(";TYPE:FILL") != std::string::npos)
        {
            return SliceLineType::Infill;
        }
        else if (strLineType.find(";TYPE:PRIME-TOWER") != std::string::npos)
        {
            return SliceLineType::PrimeTower;
        }
        else if (strLineType.find(";TYPE:Slow-Flow-Types") != std::string::npos)
        {
            return SliceLineType::FlowTravel;
        }
        else if (strLineType.find(";TYPE:Flow-In-Advance-Types") != std::string::npos)
        {
            return SliceLineType::AdvanceTravel;
        }
        else
		{
			return SliceLineType::NoneType;
        }
    }

	inline std::string str_trimmed(const std::string& src)
	{
        std::string des = src;
		int lPos = src.find_first_not_of(' ');
		int rPos = src.find_last_not_of(' ');
        if (lPos>=0 && rPos>=0)
        {
            des = src.substr(lPos, rPos - lPos + 1);
        }

		int lPos2 = src.find_first_not_of('\n');
		int rPos2 = src.find_last_not_of('\n');
        if (lPos2 >= 0 && rPos2 >= 0)
        {
            return des.substr(lPos2, rPos2 - lPos2 + 1);
        }
        return des;
	}


    GCodeStruct::GCodeStruct()
        : tempCurrentType(SliceLineType::NoneType)
        , tempNozzleIndex(0)
        , tempCurrentE(0.0f)
        , tempSpeed(0.0f)
        , layerNumberParseSuccess(true)
        , m_tracer(nullptr)
    {
        m_positions.push_back(tempCurrentPos);
    }

    GCodeStruct::~GCodeStruct()
    {
    }

    void GCodeStruct::processLayer(const std::string& layerCode, int layer, std::vector<int>& stepIndexMap)
    {
		std::vector<std::string> layerLines = stringutil::splitString(layerCode,"\n"); //layerCode.split("\n");

        int startNumber = (int)m_moves.size();
        if (layerNumberParseSuccess && layerLines.size() > 0)
        {
			std::vector<std::string> layerNumberStr = stringutil::splitString(layerLines[0],":");
            if (layerNumberStr.size() > 1)
            {
				int index = std::stoi(layerNumberStr[1].c_str());
                if (layerNumberParseSuccess)
                    tempBaseInfo.layerNumbers.push_back(index);
            }
            else
            {
                layerNumberParseSuccess = false;
            }
        }
        else
        {
            layerNumberParseSuccess = false;
        }

        //获取层高
        checkoutLayerHeight(layerLines);

        int nIndex = 0;
        for (const std::string& layerLine : layerLines)
        {
			std::string stepCode = str_trimmed(layerLine);  //" \n\r\t"
            if (stepCode.empty())
                continue;

            checkoutLayerInfo(layerLine, layer);
            processStep(stepCode, nIndex, stepIndexMap);
            ++nIndex;
        }

        tempBaseInfo.steps.push_back((int)m_moves.size() - startNumber);
    }

    void GCodeStruct::checkoutFan(const std::string& stepCode)
    {
        //std::vector <GcodeTemperature> m_temperatures;
        if (stepCode.find("M106") != std::string::npos
            || stepCode.find("M107") != std::string::npos)
        {
            GcodeFan gcodeFan = m_fans.size() > 0 ? m_fans.back() : GcodeFan();
			std::vector<std::string> strs = stringutil::splitString(stepCode," ");

            float speed = 0.0f;
            int type = 0;
            int Mtypes = -1;  //1 M106 , 2 M107
            for (const auto& it3 : strs)
            {
                auto componentStr = str_trimmed(it3);
                if (componentStr.empty())
                    continue;

                if (componentStr[0] == 'S')
                {
					speed = std::atof(componentStr.substr(1).c_str());//componentStr.mid(1).toFloat();
                }

                if (Mtypes == 1)//M106
                {
                    if (!it3.compare("P1"))
                        type = 1;
                    else if (!it3.compare("P2"))
                        type = 2;
                }
                else if (Mtypes == 2)//M107
                {
                    if (!it3.compare("P1"))
                        type = 4;
                    else if (!it3.compare("P2"))
                        type = 5;
                }

                if (!it3.compare("M106"))
                {
                    Mtypes = 1;
                }
                else if (!it3.compare("M107"))
                {
                    Mtypes = 2;
                }
            }

            if (Mtypes == 1)//M106
            {
                if (type == 1)
                    gcodeFan.camberSpeed = speed;
                else if (type == 2)
                    gcodeFan.fanSpeed_1 = speed;
                else
                    gcodeFan.fanSpeed = speed;
            }
            else if (Mtypes == 2)//M107
            {
                if (type == 4)
                    gcodeFan.camberSpeed = 0;
                else if (type == 5)
                    gcodeFan.fanSpeed_1 = 0;
                else
                    gcodeFan.fanSpeed = 0;
            }

            if (Mtypes >= 0)
            {
                m_fans.push_back(gcodeFan);
            }
        }

    }

    void GCodeStruct::checkoutTemperature(const std::string& stepCode)
    {
        //std::vector <GcodeTemperature> m_temperatures;
        if (stepCode.find("M140") != std::string::npos
            || stepCode.find("M190") != std::string::npos
            || stepCode.find("M104") != std::string::npos
            || stepCode.find("M109") != std::string::npos
            || stepCode.find("M141") != std::string::npos
            || stepCode.find("M191") != std::string::npos
            || stepCode.find("EXTRUDER_TEMP") != std::string::npos
            || stepCode.find("BED_TEMP") != std::string::npos
            )
        {
            GcodeTemperature gcodeTemperature= m_temperatures.size()>0 ? m_temperatures.back(): GcodeTemperature();
			std::vector<std::string> strs = stringutil::splitString(stepCode, " ");

            float temperature = 0.0f;
            int type = -1;
            for (const auto& it3 : strs)
            {
				auto componentStr = str_trimmed(it3);
                if (componentStr.empty())
                    continue;

                if (componentStr[0] == 'S')
                {
					temperature = std::atof(componentStr.substr(1).c_str());//temperature = componentStr.mid(1).toFloat();
                }
                else if (componentStr.find("EXTRUDER_TEMP") != std::string::npos)
                {
					std::vector<std::string> strs = stringutil::splitString(componentStr, "=");//QStringList strs = componentStr.split("=");
                    if (strs.size() >= 2)
                    {
						temperature = std::atof(strs[1].c_str()); //strs[1].toFloat();
                    }
                    gcodeTemperature.temperature = temperature;
                    type = 4;
                }
                else if (componentStr.find("BED_TEMP") != std::string::npos)
                {
                    
					std::vector<std::string> strs = stringutil::splitString(componentStr, "=");//QStringList strs = componentStr.split("=");
                    if (strs.size() >= 2)
                    {
						temperature = std::atof(strs[1].c_str()); //strs[1].toFloat();
                    }
                    gcodeTemperature.bedTemperature = temperature;
                    type = 4;
                }

                if (!it3.compare("M140")
                    || !it3.compare("M190"))
                {
                    type = 0;
                }
                else if (!it3.compare("M104")
                    || !it3.compare("M109") )
                {
                    if (!it3.compare("T1")
                        || !it3.compare("t1"))
                        type = 1;
                    else
                        type = 2;
                }
                else if (!it3.compare("M141")
                    || !it3.compare("M191"))
                {
                    type = 3;
                }
            }
            switch (type)
            {
            case 0:
                gcodeTemperature.bedTemperature = temperature;
                break;
            case 1:
                //gcodeTemperature.temperature0 = temperature;
                //break;
            case 2:
                gcodeTemperature.temperature = temperature;
                break;
            case 3:
                gcodeTemperature.camberTemperature = temperature;
                break;
            case 4:
            default:
                break;
            }

            //过滤温度为0 的情况
            if (type >= 0 && temperature > 0.0f)
            {
                if (!m_temperatures.empty())
                {
                    bool isRepeat = false;
                    for (int i=0; i< m_temperatures.size(); i++)
                    {
                        if (m_temperatures[i].bedTemperature == gcodeTemperature.bedTemperature
                            && m_temperatures[i].camberTemperature == gcodeTemperature.camberTemperature
                            && m_temperatures[i].temperature == gcodeTemperature.temperature)
                        {
                            tempTempIndex = i;
                            isRepeat = true;
                        }
                    }
                    if (!isRepeat)
                    {
                        tempTempIndex = m_temperatures.size();
                        m_temperatures.push_back(gcodeTemperature);
                    }
                }
                else
                {
                    tempTempIndex = m_temperatures.size();
                    m_temperatures.push_back(gcodeTemperature);
                }
            }
        }

    }

    void GCodeStruct::checkoutLayerInfo(const std::string& stepCode, int layer)
    {
        //;TIME_ELAPSED:548.869644
        if (stepCode.find("TIME_ELAPSED") != std::string::npos)
        {
            
			std::vector<std::string> strs = stringutil::splitString(stepCode, ":");
            if (strs.size() >= 2)
            {
				float temp = std::atof(strs[1].c_str()) - tempCurrentTime;
                float templog = 0.0f;
                //if (temp >0)
                //{
                //    templog = std::log(temp);
                //}
                //m_layerTimeLogs.insert(std::pair<int, float>(layer, templog));
                m_layerTimes.insert(std::pair<int,float>(layer, temp));
				tempCurrentTime = std::atof(strs[1].c_str());//strs[1].toFloat();
            }
        }
    }


	void GCodeStruct::checkoutLayerHeight(const std::vector<std::string>& layerLines)
	{
		float height = tempCurrentZ <= 0 ? 999 : tempCurrentZ;
		std::vector<float> heights;
		belowZ;//上层层高

		if (belowZ < tempCurrentZ)
		{
			heights.push_back(tempCurrentZ);
		}
		for (auto stepCode : layerLines)
		{
			if (stepCode.size() > 3)
			{
				std::vector<std::string> G01Strs = stringutil::splitString(stepCode, " ");
				for (const std::string& it3 : G01Strs)
				{
					std::string componentStr = str_trimmed(it3);
					if (componentStr.empty())
						continue;
					if (componentStr[0] == 'Z')
					{
						tempCurrentZ = std::atof(componentStr.substr(1).c_str());
						heights.push_back(tempCurrentZ);
					}
				}
			}
		}

		for (auto h : heights)
		{
			height = std::min(height, h);
		}

	    GcodeLayerInfo  gcodeLayerInfo = m_gcodeLayerInfos.size() > 0 ? m_gcodeLayerInfos.back() : GcodeLayerInfo();
		gcodeLayerInfo.layerHight = height + 0.00001f - belowZ;
		m_gcodeLayerInfos.push_back(gcodeLayerInfo);
		belowZ = height;
	}


  //  void GCodeStruct::checkoutLayerHeight(const std::vector<std::string>& layerLines)
  //  {
  //      float height= tempCurrentZ;

  //      bool haveG123 = false;
  //      bool haveXYZ = false;
  //      bool haveE = false;
  //      bool isHeight = false;
		//std::vector<float> heights;

		//auto addHeight = [&]() {
		//	GcodeLayerInfo  gcodeLayerInfo = m_gcodeLayerInfos.size() > 0 ? m_gcodeLayerInfos.back() : GcodeLayerInfo();

		//	float layerHight = height - belowZ;
		//	gcodeLayerInfo.layerHight = layerHight + 0.00001f;
		//	m_gcodeLayerInfos.push_back(gcodeLayerInfo);

		//	belowZ = height;
		//};

  //      for (auto stepCode : layerLines)
  //      {
  //          if (stepCode.size() > 3)
  //          {
  //              if (stepCode[0] == 'G' &&
  //                  (stepCode[1] == '0' || stepCode[1] == '1' || stepCode[1] == '2' || stepCode[1] == '3'))
  //              {
  //                  haveG123 = false;
  //                  haveXYZ = false;
  //                  haveE = false;
		//			std::vector<std::string> G01Strs = stringutil::splitString(stepCode," ");
  //                  for (const std::string& it3 : G01Strs)
  //                  {
		//				std::string componentStr = str_trimmed(it3);
  //                      if (componentStr.empty())
  //                          continue;
  //                      if (componentStr[0] == 'Z')
  //                      {
  //                          tempCurrentZ =std::atof(componentStr.substr(1).c_str());

  //                          if (!isHeight)
  //                              height = std::atof(componentStr.substr(1).c_str());

		//					heights.push_back(height);
  //                      }
  //                      
  //                      if (isHeight)
  //                          continue;
  //                          
  //                      if (componentStr[0] == 'X'|| componentStr[0] == 'Y' || componentStr[0] == 'Z')
  //                      {
  //                          haveXYZ = true;
  //                      }
  //                      else if (componentStr[0] == 'E')
  //                      {
  //                          haveE = true;
  //                      }
  //                      else if (componentStr == "G1" || componentStr == "G2" || componentStr == "G3")
  //                      {
  //                          haveG123 = true;
  //                      }
		//				if (haveG123 && haveXYZ && haveE) {
		//					addHeight();
		//					//return;
		//				}
  //                  }
  //              }
  //          }
  //      }
  // //     if (isHeight)
  // //     {
		//	//addHeight();
  // //     }
		//if ((height - belowZ) >0)
		//{
		//	int maxH = 0;
		//	for(auto h : heights)
		//	{
		//		height = std::min(height,h);
		//		maxH = std::max(height, h);
		//	}
		//	addHeight();

		//	belowZ = maxH;
		//} 
		//
  //  }

    void GCodeStruct::processPrefixCode(const std::string& stepCod)
    {
		std::vector<std::string> layerLines = stringutil::splitString(stepCod, "\n"); //QStringList layerLines = stepCod.split("\n");
        

        for (const std::string& layerLine : layerLines)
        {
			std::string stepCode = str_trimmed(layerLine);  //" \n\r\t"
            if (stepCode.empty())
                continue;

            if (stepCode.find("M140") != std::string::npos
                || stepCode.find("M190") != std::string::npos
                || stepCode.find("M104") != std::string::npos
                || stepCode.find("M109") != std::string::npos
                || stepCode.find("M141") != std::string::npos
                || stepCode.find("M191") != std::string::npos
                || stepCode.find("EXTRUDER_TEMP") != std::string::npos
                || stepCode.find("BED_TEMP") != std::string::npos
                || stepCode.find("M106") != std::string::npos
                || stepCode.find("M107") != std::string::npos )
            {
                checkoutFan(stepCode);
                checkoutTemperature(stepCode);
            }
            else if (stepCode.find("Outer Wall Speed") != std::string::npos
                || stepCode.find("Inner Wall Speed") != std::string::npos
                || stepCode.find("Infill Speed") != std::string::npos
                || stepCode.find("Top/Bottom Speed") != std::string::npos
                || stepCode.find("Initial Layer Speed") != std::string::npos
                || stepCode.find("Skirt/Brim Speed") != std::string::npos
                || stepCode.find("Prime Tower Speed") != std::string::npos
                )
            {
                //获取速度最大限制
				std::vector<std::string> strs = stringutil::splitString(stepCod, ":");//stepCode.split(":");
                if (strs.size() == 2)
                {
					std::string componentStr = str_trimmed(strs[1]);
                    if (componentStr.empty())
                        continue;
					float speed = tempSpeedMax;
					float component = std::atof(componentStr.c_str()) * 60;
					std::max(speed, component);
                }
            }
        }
    }

    void GCodeStruct::processStep(const std::string& stepCode, int nIndex, std::vector<int>& stepIndexMap)
    {
        checkoutTemperature(stepCode);
        checkoutFan(stepCode);

        if (stepCode.find(";TYPE:") != std::string::npos && stepCode.find(";TYPE:end") == std::string::npos)
        {
            tempCurrentType = GetLineType(stepCode);
        }
        if (stepCode.at(0) == 'T')
        {
			std::string nozzleIndexStr = stepCode.substr(1);
            if (nozzleIndexStr.size() > 0)
            {
                //bool success = false;
				int index = std::atoi(nozzleIndexStr.c_str());
                //if (success)
                    tempNozzleIndex = index;
            }

            if (tempBaseInfo.nNozzle < tempNozzleIndex + 1)
                tempBaseInfo.nNozzle = tempNozzleIndex + 1;
        }
        else if (stepCode[0] == 'G' && stepCode.size() > 1)
        {
            if (stepCode[1] == '2' || stepCode[1] == '3')
            {
                processG23(stepCode, nIndex, stepIndexMap);
            }
            else if (stepCode[1] == '0' || stepCode[1] == '1')
            {
                processG01(stepCode, nIndex, stepIndexMap);
            }
            else if (stepCode.find("G92") != std::string::npos)
            {
                tempCurrentE = 0.0f;
            }
        }
    }

    void GCodeStruct::processG01(const std::string& G01Str, int nIndex, std::vector<int>& stepIndexMap, bool isG2G3)
    {
        std::vector<std::string> G01Strs = stringutil::splitString(G01Str," ");

        trimesh::vec3 tempEndPos = tempCurrentPos;
        double tempEndE = tempCurrentE;
        SliceLineType tempType = tempCurrentType;
        bool havaXYZ = false;

        GcodeLayerInfo gcodeLayerInfo = m_gcodeLayerInfos.size() > 0 ? m_gcodeLayerInfos.back() : GcodeLayerInfo();
        for (const std::string& it3 : G01Strs)
        {
            std::string componentStr = str_trimmed(it3);
            //it4 ==G1 / F4800 / X110.125 / Y106.709 /Z0.6 /E575.62352
            if (componentStr.empty())
                continue;

            if (componentStr[0] == 'F')
            {
                tempSpeed = std::atof(componentStr.substr(1).c_str());
            }
            else if (componentStr[0] == 'E' || componentStr[0] == 'P')
            {
				double e = std::atof(componentStr.substr(1).c_str());
                if (parseInfo.relativeExtrude)
                    tempEndE += e;
                else
                    tempEndE = e;
            }
            else if (componentStr[0] == 'X')
            {
				tempEndPos.at(0) = std::atof(componentStr.substr(1).c_str());
                havaXYZ = true;
            }
            else if (componentStr[0] == 'Y')
            {
				tempEndPos.at(1) = std::atof(componentStr.substr(1).c_str());
                havaXYZ = true;
            }
            else if (componentStr[0] == 'Z')
            {
                tempEndPos.at(2) = std::atof(componentStr.substr(1).c_str());;
                havaXYZ = true;
            }
        }

        if (tempEndE == tempCurrentE)
        {
            if (G01Str[1] == '0' || havaXYZ)
            {
                tempType = SliceLineType::Travel;
            }
        }
        else if (tempEndE < tempCurrentE)
        {
            if (havaXYZ)
                tempType = SliceLineType::Travel;
            else
            {
                tempType = SliceLineType::React;
                m_retractions.push_back(m_positions.size()-1);
            }
                
        }

        if (havaXYZ)
        {
            int index = (int)m_positions.size();
            m_positions.push_back(tempEndPos);
            GCodeMove move;
            move.start = index - 1;
            //limit speed
            if (tempSpeedMax > 0.0f)
            {
                tempSpeed = tempSpeed > tempSpeedMax ? tempSpeedMax : tempSpeed;
            }
            move.speed = tempSpeed;
            move.e = tempEndE - tempCurrentE;
            move.type = tempType;
            if (move.e == 0.0f)
                move.type = SliceLineType::Travel;
            else if (move.e >0.0f && move.type==SliceLineType::OuterWall && m_moves.back().type==SliceLineType::Travel)
            {
                m_zSeams.push_back(move.start);
            }

            move.extruder = tempNozzleIndex;
            m_moves.emplace_back(move);

            //add temperature fan and time ...
            if (m_temperatures.empty())
            {
                tempTempIndex = m_temperatures.size();
                m_temperatures.push_back(GcodeTemperature());
            }
            if (m_fans.empty())
            {
                m_fans.push_back(GcodeFan());
            }
            if (m_gcodeLayerInfos.empty())
            {
                m_gcodeLayerInfos.push_back(GcodeLayerInfo());
            }

            //float material_density = parseInfo.material_density;
            float material_diameter = parseInfo.material_diameter;
            if (!isG2G3 && SliceLineType::Travel != tempType && SliceLineType::React != tempType)
            {
                //calculate width
                trimesh::vec3 offset = tempCurrentPos - tempEndPos;
                offset.z = 0;
                float len = trimesh::length(offset);
                float h = m_gcodeLayerInfos.back().layerHight;

                float material_s = PI * (material_diameter * 0.5) * (material_diameter * 0.5);
                float width = 0.0f;
                if (len != 0 && h != 0 && move.e > 0.0f)
                {
                    width = move.e * material_s / len / h;
                }
                if (len < 0.05)
                {
                    width = m_gcodeLayerInfos.back().width;
                }

                //calculate flow
                float flow = 0.0f;
                if (move.e > 0.0f && len > 0)
                {
                    float r = material_diameter / 2;
                    flow = r* r* PI* move.e * move.speed / 60.0 / len;
                }

                if ((std::abs(m_gcodeLayerInfos.back().width - width) > 0.001 && width > 0)
                    || (std::abs(m_gcodeLayerInfos.back().flow - flow) > 0.001 && flow > 0))
                {
                    GcodeLayerInfo  gcodeLayerInfo = m_gcodeLayerInfos.size() > 0 ? m_gcodeLayerInfos.back() : GcodeLayerInfo();
                    gcodeLayerInfo.width = width;
                    gcodeLayerInfo.flow = flow;
                    m_gcodeLayerInfos.push_back(gcodeLayerInfo);
                }
                //end
            }
 
            m_temperatureIndex.push_back(m_temperatures.size()-1);
            m_fanIndex.push_back(m_fans.size() - 1);
            m_layerInfoIndex.push_back(m_gcodeLayerInfos.size() - 1);
            //end

            stepIndexMap.push_back(nIndex);

            tempBaseInfo.gCodeBox += tempEndPos;

            if(tempType != SliceLineType::Travel)
                processSpeed(tempSpeed);
        }

        tempCurrentPos = tempEndPos;
        tempCurrentE = tempEndE;
    }

    void GCodeStruct::processG23(const std::string& G23Code, int nIndex, std::vector<int>& stepIndexMap)
    {
        std::vector<std::string> G23Strs = stringutil::splitString(G23Code," ");//G1 Fxxx Xxxx Yxxx Exxx
        //G3 F1500 X118.701 Y105.96 I9.55 J1.115 E7.96039
        bool isG2 = true;
        if (G23Code[1] == '3')
            isG2 = false;

        bool bIsTravel = true;
        float f = 0.0f;
        float e = 0.0f;
        float x = 0.0f;
        float y = 0.0f;
        float i = 0.0f;
        float j = 0.0f;
        bool  bcircles = false;
        float currentE = tempCurrentE;
        for (const std::string& it3 : G23Strs)
        {
            //it4 ==G1 / F4800 / X110.125 / Y106.709 /Z0.6 /E575.62352
            std::string G23Str = str_trimmed(it3);
            if (G23Str.empty())
                continue;


			float currentValue = std::atof(G23Str.substr(1).c_str());
            if (G23Str[0] == 'E')
            {
				float _e = currentValue;//G23Str.mid(1).toFloat();
                if (_e > 0)
                {
                    if (!parseInfo.relativeExtrude)
                        e = _e - tempCurrentE;
                    else
                        e = _e;
                    bIsTravel = false;
                }

                currentE = _e;
            }
            else if (G23Str[0] == 'F')
            {
				f = currentValue;
                tempSpeed = currentValue;
            }
            else if (G23Str[0] == 'X')
            {
                x = currentValue;
            }
            else if (G23Str[0] == 'Y')
            {
                y = currentValue;
            }
            else if (G23Str[0] == 'I')
            {
                i = currentValue;
            }
            else if (G23Str[0] == 'J')
            {
                j = currentValue;
            }
            else if (G23Str[0] == 'P')
            {
                bcircles = true;
                bIsTravel = true;
                if (x == 0 && y == 0)
                {
                    x = tempCurrentPos.x;
                    y = tempCurrentPos.y;
                }
            }
        }

        trimesh::vec3 circlePos = tempCurrentPos;
        circlePos.x += i;
        circlePos.y += j;
        trimesh::vec3 circleEndPos = tempCurrentPos;
        circleEndPos.x = x;
        circleEndPos.y = y;

        float theta = 0.0f;
        std::vector<trimesh::vec> out;
        if (isG2)
        {
            theta = mmesh::getAngelOfTwoVector(tempCurrentPos, circleEndPos, circlePos);
        }
        else
        {
            theta = mmesh::getAngelOfTwoVector(circleEndPos, tempCurrentPos, circlePos);
        }
        if (bcircles)
        {
            theta = 360;
        }

        trimesh::vec3 offset = tempCurrentPos - circlePos;
        offset.z = 0;
        float radius = trimesh::length(offset);
        //float material_density = parseInfo.material_density;
        float material_diameter = parseInfo.material_diameter;
        float material_s = PI * (material_diameter * 0.5) * (material_diameter * 0.5);
        //计算弧长
        float len = theta * M_PIf * radius / 180.0;
        float r = material_diameter / 2.0f;
        float flow = r * r * PI * e * tempSpeed / 60.0 / len;

        float h = m_gcodeLayerInfos.back().layerHight;
        float width = 0.0f;
        if (len != 0 && h != 0 && e > 0.0f)
        {
            width = e * material_s / len / h;
        }

        if (std::abs(m_gcodeLayerInfos.back().flow - flow) > 0.001 && len >= 1.0f)
        {
            GcodeLayerInfo  gcodeLayerInfo = m_gcodeLayerInfos.size() > 0 ? m_gcodeLayerInfos.back() : GcodeLayerInfo();
            gcodeLayerInfo.width = width;
            gcodeLayerInfo.flow = flow;
            m_gcodeLayerInfos.push_back(gcodeLayerInfo);
        }


        mmesh::getDevidePoint(circlePos, tempCurrentPos, out, theta, isG2);
        out.push_back(circleEndPos);
        float devideE = e;
        if (out.size() > 0)
        {
            devideE = e / out.size();
        }

		std::list<std::string> G23toG1s;
        for (size_t ii = 0; ii < out.size(); ii++)
        {
            std::string devideTemp = "";
            if (bIsTravel)
            {
                devideTemp += "G0 ";
            }
            else
            {
                devideTemp += "G1 ";
            }
            if (f)
            {
                char itc[20];
                sprintf(itc, "F%0.1f ", f);
                devideTemp += itc;
            }
            char itcx[20];
            sprintf(itcx, "X%0.3f ", out[ii].x);
            devideTemp += itcx;
            char itcy[20];
            sprintf(itcy, "Y%0.3f ", out[ii].y);
            devideTemp += itcy;
            if (!bIsTravel)
            {
                char itce[20];
                double ce = parseInfo.relativeExtrude ? devideE : tempCurrentE + devideE * (ii + 1);
                sprintf(itce, "E%0.5f", ce);
                devideTemp += itce;
            }

            G23toG1s.push_back(devideTemp.c_str());
        }

        for (const auto& G23toG01 : G23toG1s)
        {
            processG01(G23toG01, nIndex, stepIndexMap,true);
        }

        tempCurrentE = currentE;
    }

    void GCodeStruct::processSpeed(float speed)
    {
        if (tempBaseInfo.speedMax < speed)
            tempBaseInfo.speedMax = speed;
        if (tempBaseInfo.speedMin > speed)
            tempBaseInfo.speedMin = speed;
    }

	void GCodeStruct::buildFromResult(gcode::SliceResult* result, ccglobal::Tracer* tracer)
	{
		cxsw::parseGCodeInfo(result, parseInfo);

		//get temperature and fan
		processPrefixCode(result->prefixCode());

		tempBaseInfo.nNozzle = 1;
		int layer = 0;
		int layerCount = (int)result->layerCode().size();
		for (const auto& it : result->layerCode())
		{
			std::vector<int> stepIndexMap;
			std::string layerCode = str_trimmed(it);
			processLayer(layerCode, layer, stepIndexMap);
			++layer;

			if (m_tracer)
			{
				m_tracer->progress((float)layer / (float)layerCount);
				if (m_tracer->interrupt())
				{
					m_tracer->failed("GCodeStruct::buildFromResult interrupt.");
					layerNumberParseSuccess = false;
					break;
				}
			}
		}

		tempBaseInfo.totalSteps = (int)m_moves.size();
		tempBaseInfo.layers = (int)tempBaseInfo.layerNumbers.size();
		if (!layerNumberParseSuccess)
		{
			tempBaseInfo.layerNumbers.clear();
			tempBaseInfo.layers = 1;
			tempBaseInfo.steps.clear();
			tempBaseInfo.steps.push_back(tempBaseInfo.totalSteps);
		}


		for (GCodeMove& move : m_moves)
		{
			move.speed = move.speed / tempBaseInfo.speedMax;
		}

		{
			float minFlow = FLT_MAX, maxFlow = FLT_MIN;
			float minWidth = FLT_MAX, maxWidth = FLT_MIN;
			float minHeight = FLT_MAX, maxHeight = FLT_MIN;

			for (GcodeLayerInfo& info : m_gcodeLayerInfos)
			{
				if (info.flow > 0.0)
				{
					minFlow = fminf(info.flow, minFlow);
					maxFlow = fmaxf(info.flow, maxFlow);
				}

				if (info.width > 0.0)
				{
					minWidth = fminf(info.width, minWidth);
					maxWidth = fmaxf(info.width, maxWidth);
				}

				minHeight = fminf(info.layerHight, minHeight);
				maxHeight = fmaxf(info.layerHight, maxHeight);
			}
			tempBaseInfo.minFlowOfStep = minFlow;
			tempBaseInfo.maxFlowOfStep = maxFlow;
			tempBaseInfo.minLineWidth = minWidth;
			tempBaseInfo.maxLineWidth = maxWidth;
			tempBaseInfo.minLayerHeight = minHeight;
			tempBaseInfo.maxLayerHeight = maxHeight;

		}

		{
			float minTime = FLT_MAX, maxTime = FLT_MIN;
			for (auto t : m_layerTimes)
			{
				float time = t.second;
				minTime = fminf(time, minTime);
				maxTime = fmaxf(time, maxTime);
			}
			tempBaseInfo.minTimeOfLayer = minTime;
			tempBaseInfo.maxTimeOfLayer = maxTime;
		}


		{
			float minTemp = FLT_MAX, maxTemp = FLT_MIN;
			for (GcodeTemperature& t : m_temperatures)
			{
				minTemp = fminf(t.temperature, minTemp);
				maxTemp = fmaxf(t.temperature, maxTemp);
			}
			tempBaseInfo.minTemperature = minTemp;
			tempBaseInfo.maxTemperature = maxTemp;
		}
	}

	void GCodeStruct::buildFromResult(SliceResultPointer result, const GCodeParseInfo& info, GCodeStructBaseInfo& baseInfo, std::vector<std::vector<int>>& stepIndexMaps, ccglobal::Tracer* tracer /*= nullptr*/)
	{
		m_tracer = tracer;

		//get temperature and fan
		processPrefixCode(result->prefixCode());

		parseInfo = info;
		tempBaseInfo.nNozzle = 1;
		int layer = 0;
		int layerCount = (int)result->layerCode().size();
		for (const auto& it : result->layerCode())
		{
			std::vector<int> stepIndexMap;
			std::string layerCode = str_trimmed(it);
			processLayer(layerCode, layer, stepIndexMap);
			stepIndexMaps.push_back(stepIndexMap);
			++layer;

			if (m_tracer)
			{
				m_tracer->progress((float)layer / (float)layerCount);
				if (m_tracer->interrupt())
				{
					m_tracer->failed("GCodeStruct::buildFromResult interrupt.");
					layerNumberParseSuccess = false;
					break;
				}
			}
		}

		tempBaseInfo.totalSteps = (int)m_moves.size();
		tempBaseInfo.layers = (int)tempBaseInfo.layerNumbers.size();
		if (!layerNumberParseSuccess)
		{
			tempBaseInfo.layerNumbers.clear();
			tempBaseInfo.layers = 1;
			tempBaseInfo.steps.clear();
			tempBaseInfo.steps.push_back(tempBaseInfo.totalSteps);
			if (stepIndexMaps.size() > 0)
			{
				stepIndexMaps.resize(1);
			}
		}


		for (GCodeMove& move : m_moves)
		{
			move.speed = move.speed / tempBaseInfo.speedMax;
		}

		{
			float minFlow = FLT_MAX, maxFlow = FLT_MIN;
			float minWidth = FLT_MAX, maxWidth = FLT_MIN;
			float minHeight = FLT_MAX, maxHeight = FLT_MIN;

			for (GcodeLayerInfo& info : m_gcodeLayerInfos)
			{
				if (info.flow > 0.0)
				{
					minFlow = fminf(info.flow, minFlow);
					maxFlow = fmaxf(info.flow, maxFlow);
				}

				if (info.width > 0.0)
				{
					minWidth = fminf(info.width, minWidth);
					maxWidth = fmaxf(info.width, maxWidth);
				}

				minHeight = fminf(info.layerHight, minHeight);
				maxHeight = fmaxf(info.layerHight, maxHeight);
			}
			tempBaseInfo.minFlowOfStep = minFlow;
			tempBaseInfo.maxFlowOfStep = maxFlow;
			tempBaseInfo.minLineWidth = minWidth;
			tempBaseInfo.maxLineWidth = maxWidth;
			tempBaseInfo.minLayerHeight = minHeight;
			tempBaseInfo.maxLayerHeight = maxHeight;

		}

		{
			float minTime = FLT_MAX, maxTime = FLT_MIN;
			for (auto t : m_layerTimes)
			{
				float time = t.second;
				minTime = fminf(time, minTime);
				maxTime = fmaxf(time, maxTime);
			}
			tempBaseInfo.minTimeOfLayer = minTime;
			tempBaseInfo.maxTimeOfLayer = maxTime;
		}


		{
			float minTemp = FLT_MAX, maxTemp = FLT_MIN;
			for (GcodeTemperature& t : m_temperatures)
			{
                if (t.temperature > 0)
                {
					minTemp = fminf(t.temperature, minTemp);
					maxTemp = fmaxf(t.temperature, maxTemp);
				}
			}
			tempBaseInfo.minTemperature = minTemp;
			tempBaseInfo.maxTemperature = maxTemp;
		}
		baseInfo = tempBaseInfo;
	}

    void GCodeStruct::getPathData(const trimesh::vec3 point, float e, int type)
    {
        trimesh::vec3 tempEndPos = tempCurrentPos;
        double tempEndE = tempCurrentE;

        if (parseInfo.relativeExtrude)
            tempEndE += e;
        else
            tempEndE = e;

        if (point.z >= 0 )
        {
            tempEndPos = point;
            tempEndPos = { tempEndPos.x / 1000.f,tempEndPos.y / 1000.f ,tempEndPos.z / 1000.f };
        }
        else
        {
            tempEndPos.x = point.x;
            tempEndPos.y = point.y;
            tempEndPos = { tempEndPos.x / 1000.f,tempEndPos.y / 1000.f ,tempEndPos.z};
        }

        int index = (int)m_positions.size();
        m_positions.push_back(tempEndPos);
        GCodeMove move;
        move.type = (SliceLineType)type;
        move.start = index - 1;
        move.speed = tempSpeed;

        m_moves.emplace_back(move);

        tempCurrentPos = tempEndPos;
        tempCurrentE = tempEndE;
    }

    void GCodeStruct::setParam(gcode::GCodeParseInfo& pathParam)
    {
        parseInfo = pathParam;
    }

    void GCodeStruct::setLayer(int layer)
    {}
    void GCodeStruct::setSpeed(float s)
    {
        tempSpeed = s;
    }
    void GCodeStruct::setTEMP(float temp)
    {}
    void GCodeStruct::setExtruder(int nr)
    {}
    void GCodeStruct::setFan(float fan)
    {}
}

