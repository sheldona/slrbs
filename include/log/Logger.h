#pragma once

//#include "spdlog/spdlog.h"
#include <algorithm>
#include <any>
#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <vector>

class Logger
{
public:
    Logger();

    virtual ~Logger();

    int addField(const std::string& _name)
    {
        const auto it = std::find(m_fieldNames.begin(), m_fieldNames.end(), _name);
        
        assert(it == m_fieldNames.end());

        const int idx = m_fieldNames.size();
        m_fieldNames.push_back(_name);
        m_buf.push_back(std::vector<std::any>());
        m_buf.back().reserve(256 * 1024);
        return idx;
    }

    void clear()
    {
        for (auto& buf : m_buf)
        {
            buf.clear();
        }
    }

    template<typename T>
    void pushVal(int _idx, const T& _val)
    {
        assert(0 <= _idx && _idx < m_buf.size());

        m_buf[_idx].push_back(_val);
    }

    virtual void save(const std::string& _filename) { }

protected:

    std::vector<std::vector<std::any>> m_buf;
    std::vector<std::string> m_fieldNames;

};
