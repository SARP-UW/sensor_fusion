#include "DataIngestion.hpp"
#include <iostream>
#include <fstream>

// Magic Bytes (Must match DataGenerator.cpp).
const uint8_t ID_IMU = 0x10;
const uint8_t ID_BARO = 0x20;
const uint8_t ID_GPS = 0x30;

ParsedData DataIngestion::loadFromFile(const std::string &filepath)
{
  ParsedData data;

  // Open the file in Binary Mode.
  std::ifstream file(filepath, std::ios::binary);
  if (!file.is_open())
  {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return data;
  }

  std::cout << "Parsing file: " << filepath << "..." << std::endl;

  // The Parsing Loop.
  uint8_t packet_header;

  // Read until we hit EOF.
  while (file.read(reinterpret_cast<char *>(&packet_header), sizeof(packet_header)))
  {
    // Check which packet type we found.
    if (packet_header == ID_IMU)
    {
      ImuMeasurement imu;
      file.read(reinterpret_cast<char *>(&imu), sizeof(ImuMeasurement));
      data.imu_data.push_back(imu);
    }
    else if (packet_header == ID_BARO)
    {
      BaroMeasurement baro;
      file.read(reinterpret_cast<char *>(&baro), sizeof(BaroMeasurement));
      data.baro_data.push_back(baro);
    }
    else if (packet_header == ID_GPS)
    {
      GpsMeasurement gps;
      file.read(reinterpret_cast<char *>(&gps), sizeof(GpsMeasurement));
      data.gps_data.push_back(gps);
    }
    else
    {
      // Unknown packet type - possibly corrupted file.
      std::cerr << "Unknown Packet Header: " << (int)packet_header << " at position " << file.tellg() << std::endl;
      break;
    }
  }

  std::cout << "Parsing Complete." << std::endl;
  std::cout << "Loaded " << data.imu_data.size() << " IMU packets." << std::endl;
  std::cout << "Loaded " << data.baro_data.size() << " Baro packets." << std::endl;

  return data;
}