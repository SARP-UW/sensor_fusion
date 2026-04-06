#pragma once

#include <string>
#include <vector>
#include "SensorData.hpp"

class DataIngestion
{
public:
  ParsedData loadFromFile(const std::string &filepath);
};