#include "DataIngestion.hpp"
#include <iostream>
#include <fstream>

// Packet Headers to identify data in the file.
const uint8_t ID_IMU = 0x10;
const uint8_t ID_BARO = 0x20;
const uint8_t ID_GPS = 0x30;
const uint8_t ID_MAG = 0x40;

ParsedData DataIngestion::loadFromFile(const std::string &filepath)
{
  ParsedData data;

  // Open input file.
  std::ifstream file(filepath, std::ios::binary);
  if (!file.is_open())
  {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return data;
  }
  std::cout << "Parsing file: " << filepath << "..." << std::endl;

  // Read packets one by one.
  uint8_t packet_header;

  // Parse until end of file.
  while (file.read(reinterpret_cast<char *>(&packet_header), sizeof(packet_header)))
  {
    // Check which packet type we found.
    if (packet_header == ID_IMU)
    {
      ImuMeasurement imu;
      if (file.read(reinterpret_cast<char *>(&imu), sizeof(ImuMeasurement)))
      {
        data.imu_data.push_back(imu);
      }
    }
    else if (packet_header == ID_BARO)
    {
      BaroMeasurement baro;
      if (file.read(reinterpret_cast<char *>(&baro), sizeof(BaroMeasurement)))
      {
        data.baro_data.push_back(baro);
      }
    }
    else if (packet_header == ID_GPS)
    {
      GpsMeasurement gps;
      if (file.read(reinterpret_cast<char *>(&gps), sizeof(GpsMeasurement)))
      {
        data.gps_data.push_back(gps);
      }
    }
    else if (packet_header == ID_MAG)
    {
      MagMeasurement mag = {};
      if (file.read(reinterpret_cast<char *>(&mag), sizeof(MagMeasurement)))
      {
        data.mag_data.push_back(mag);
      }
    }
    else
    {
      // Unknown packet type - possibly corrupted file.
      std::cerr << "Unknown Packet Header: " << (int)packet_header << " at position " << file.tellg() << std::endl;
      break;
    }
  }

  // Close the file.
  file.close();
  std::cout << "Parsing Complete." << std::endl;
  std::cout << "Loaded " << data.imu_data.size() << " IMU packets." << std::endl;
  std::cout << "Loaded " << data.baro_data.size() << " Baro packets." << std::endl;
  std::cout << "Loaded " << data.gps_data.size() << " GPS packets." << std::endl;
  std::cout << "Loaded " << data.mag_data.size() << " Mag packets." << std::endl;

  return data;
}