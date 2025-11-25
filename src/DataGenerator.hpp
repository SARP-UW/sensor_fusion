#pragma once

#include <string>
#include <vector>
#include <random> // For generating noise
#include "SensorData.hpp"

class DataGenerator
{
public:
  // Config for our simulation.
  struct Config
  {
    // Total simulation time in seconds.
    double duration_sec = 30.0;

    // Physics time step in seconds (100 Hz).
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
  std::normal_distribution<double> noise_imu_acc_{0.0, 0.2};   // Mean 0, StdDev 0.2 m/s^2.
  std::normal_distribution<double> noise_imu_gyro_{0.0, 0.01}; // Mean 0, StdDev 0.01 rad/s.
  std::normal_distribution<double> noise_baro_{0.0, 2.0};      // Mean 0, StdDev 2.0 Pa.
  std::normal_distribution<double> noise_gps_{0.0, 2.5};       // Mean 0, StdDev 2.5 meters.

  // Helper to write a specific sensor struct to the file.
  template <typename T>
  void writePacket(std::ofstream &file, uint8_t packet_type, const T &data);
};