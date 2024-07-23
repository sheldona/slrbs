#include "log/CSVLogger.h"

#include <fstream>

CSVLogger::CSVLogger() : Logger()
{
}

CSVLogger::~CSVLogger()
{

}

void CSVLogger::save(const std::string& _filename)
{
    std::ofstream fs(_filename);
    const int nrows = m_buf[0].size();
    const int ncols = m_fieldNames.size();

    for (int j = 0; j < ncols; ++j)
    {
        fs << m_fieldNames[j];
        if (j < ncols-1) fs << ",";
    }
    fs << std::endl;

    for (int i = 0; i < nrows; ++i)
    {
        for (int j = 0; j < ncols; ++j)
        {
            if (i < m_buf[j].size())
            {
                // TODO need to fix this.  Assumes all types are float. 
                fs << std::any_cast<float>(m_buf[j][i]);
            }
            
            if (j < ncols-1) fs << ",";
        }
        fs << std::endl;
    }

    fs.flush();
    fs.close();
}
