#pragma once

#include <string>
#include <vector>
#include "SensorData.hpp"

// Class to handle data ingestion from binary files.
class DataIngestion
{
public:
  ParsedData loadFromFile(const std::string &filepath);
};