#pragma once

#include "log/Logger.h"

class CSVLogger : public Logger
{
public:
    CSVLogger();

    virtual ~CSVLogger();

    virtual void save(const std::string& _filename) override;

};
