#pragma once

#include <string>
#include <vector>
#include <random>
#include "SensorData.hpp"

class DataGenerator
{
public:
  struct Config
  {
    double duration_sec = 60.0;

    double dt_sec = 0.01;
    std::string output_filename = "synthetic_flight.bin";
  };

  DataGenerator(Config config);

  void generate();

private:
  Config config_;

  std::default_random_engine generator_;
  std::normal_distribution<double> noise_imu_acc_{0.0, 0.2};
  std::normal_distribution<double> noise_imu_gyro_{0.0, 0.01};
  std::normal_distribution<double> noise_baro_{0.0, 2.0};
  std::normal_distribution<double> noise_gps_{0.0, 2.5};

  template <typename T>
  void writePacket(std::ofstream &file, uint8_t packet_type, const T &data);
};