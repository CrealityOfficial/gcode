#include "sliceresult.h"
#include "gcode/gcodehelper.h"

//#include <QtCore/QDebug>
//#include <QtCore/QStandardPaths>
//#include <QtCore/QDir>
//#include "qtusercore/string/resourcesfinder.h"

namespace gcode
{
    SliceResult::SliceResult() 
    {
        //G.reset(new USettings());
        //G->loadCompleted();
    }

    SliceResult::~SliceResult()
    {
    }

    const std::string& SliceResult::prefixCode()
    {
        if (m_data_gcodeprefix.size() > 0)
            return *m_data_gcodeprefix.begin();
        return m_emptyString;
    }

    const std::vector<std::string>& SliceResult::layerCode()
    {
        return m_data_gcodelayer;
    }

    const std::string& SliceResult::layer(int index)
    {
        if (index < 0 || index >= (int)m_data_gcodelayer.size())
            return m_emptyString;

        return m_data_gcodelayer.at(index);
    }

    const std::string& SliceResult::tailCode()
    {
        if (m_data_gcodetail.size() > 0)
            return *(m_data_gcodetail.begin());
        return m_emptyString;
    }

    void SliceResult::clear()
    {
        m_data_gcodelayer.clear();
        m_data_gcodeprefix.clear();
    }

	bool SliceResult::load(const std::string& fileName, ccglobal::Tracer* tracer)
    {
		m_fileName = fileName;
		//qDebug() << QString("cxLoadGCode : [%1]").arg(fileName);

		m_data_gcodelayer.clear();
		m_data_gcodeprefix.clear();

		//QFileInfo fileInfo(fileName);
		std::ifstream  fileInfo(fileName);
		//qint64 fileSize = fileInfo.size();

		//QFile file;
		//file.setFileName(fileName);
		if (!fileInfo.is_open())
		{
			return false;
		}

		std::string value = "";
		bool headend = false;
		long long int readBytes = 0;
		char line[1024] = { 0 };
		while (fileInfo.getline(line, sizeof(line)))
		{
			std::string temp = line;
			value += temp;

			readBytes += sizeof(line);
			if (tracer && (readBytes % 20000) == 0)
			{
				//float p = (float)((double)readBytes / (double)fileSize);
				//tracer->progress(p);

				if (tracer->interrupt())
				{
					tracer->failed("cxLoadGCode load interrupt ");
					m_data_gcodelayer.clear();
					break;
				}
			}

			if (temp.find(";LAYER_COUNT"))
			{
				if (!headend)
				{
					m_data_gcodeprefix.push_back(value);
				}
				else
				{
					//针对逐个打印 增加抬升代码
					std::string str = m_data_gcodelayer.back();
					str += value;
					m_data_gcodelayer.back() = str;
				}

				//layerString.append(temp);
				headend = true;
				value.clear();
				continue;
			}
			if (!headend)
			{
				continue;
			}
			if (temp.find(";TIME_ELAPSED:"))
			{
				m_data_gcodelayer.push_back(value);
				value.clear();
				continue;
			}
		}
		m_data_gcodetail.push_back(value);
		fileInfo.close();
		return true;
    }

	std::string SliceResult::fileName()
    {
        return m_fileName;
    }

	std::string SliceResult::sliceName()
    {
        return m_sliceName;
    }

    void SliceResult::setSliceName(const std::string& _sliceName)
    {
        m_sliceName = _sliceName;
    }
}
