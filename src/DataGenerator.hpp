#pragma once

#include <string>
#include <vector>
#include <random>
#include "SensorData.hpp"

// DataGenerator Class: Generates synthetic flight data and writes to a binary file.
class DataGenerator
{
public:
  // Config for our simulation.
  struct Config
  {
    // Total simulation time (s).
    double duration_sec = 60.0;

    // Time step at 100 Hz (s).
    double dt_sec = 0.01;
    std::string output_filename = "synthetic_flight.bin";
  };

  DataGenerator(Config config);

  // The main function that runs the physics loop and writes the file.
  void generate();

private:
  Config config_;

  // Gaussian Noise Generators.
  std::default_random_engine generator_;
  std::normal_distribution<double> noise_imu_acc_{0.0, 0.2};
  std::normal_distribution<double> noise_imu_gyro_{0.0, 0.01};
  std::normal_distribution<double> noise_baro_{0.0, 2.0};
  std::normal_distribution<double> noise_gps_{0.0, 2.5};

  // Helper to write a specific sensor struct to the file.
  template <typename T>
  void writePacket(std::ofstream &file, uint8_t packet_type, const T &data);
};