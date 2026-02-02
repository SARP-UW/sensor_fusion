#include "DataGenerator.hpp"
#include <fstream>
#include <iostream>
#include <cmath>

// Packet Headers to identify data in the file.
const uint8_t ID_IMU = 0x10;
const uint8_t ID_BARO = 0x20;
const uint8_t ID_GPS = 0x30;
const uint8_t ID_MAG = 0x40;

// Constructor.
DataGenerator::DataGenerator(Config config) : config_(config) {}

// Main Simulation Loop.
void DataGenerator::generate()
{
  // Open output file.
  std::ofstream file(config_.output_filename, std::ios::binary);
  if (!file.is_open())
  {
    std::cerr << "Failed to open output file!" << std::endl;
    return;
  }
  std::cout << "Generating synthetic flight data..." << std::endl;

  // Simulation State Variables.
  double t = 0.0;          // Current time (s).
  double pos_z = 0.0;      // Altitude (m).
  double vel_z = 0.0;      // Velocity (m/s).
  double true_acc_z = 0.0; // Kinematic acceleration (m/s^2).

  // Flight Events.
  double burn_time = 10.0;  // Engine burns for 5 seconds.
  double apogee_time = 0.0; // Will be set automatically.

  // Simple Physics Engine.
  while (t < config_.duration_sec)
  {
    if (t < burn_time)
    {
      // Phase 1: Burn.
      true_acc_z = 20.0;
    }
    else if (vel_z > 0)
    {
      // Phase 2: Coast.
      true_acc_z = -9.81;
    }
    else if (pos_z > 0)
    {
      // Phase 3: Descent.
      true_acc_z = 0.0;
      vel_z = -5.0;
    }
    else
    {
      // Phase 4: Landing.
      true_acc_z = 0.0;
      vel_z = 0.0;
      pos_z = 0.0;
    }

    // Euler Integration.
    if (pos_z > 0 || t < burn_time)
    {
      vel_z += true_acc_z * config_.dt_sec;
      pos_z += vel_z * config_.dt_sec;
    }

    // Generate Sensor Measurements.

    // IMU Generation: 100 Hz.
    ImuMeasurement imu;

    imu.timestamp_sec = t;
    imu.sensor_id = 0;
    imu.acc_x = 0.0 + noise_imu_acc_(generator_);
    imu.acc_y = 0.0 + noise_imu_acc_(generator_);
    imu.acc_z = true_acc_z + 9.81 + noise_imu_acc_(generator_);
    imu.gyro_x = 0.0 + noise_imu_gyro_(generator_);
    imu.gyro_y = 0.0 + noise_imu_gyro_(generator_);
    imu.gyro_z = 0.0 + noise_imu_gyro_(generator_);

    writePacket(file, ID_IMU, imu);

    // Barometer Generation: 10 Hz.
    if (static_cast<int>(t * 100) % 10 == 0)
    {
      BaroMeasurement baro;
      baro.timestamp_sec = t;
      baro.sensor_id = 0;

      double sea_level_pa = 101325.0;
      double true_pressure = sea_level_pa * std::pow(1.0 - (pos_z / 44330.0), 5.255);

      baro.pressure_Pa = true_pressure + noise_baro_(generator_);
      baro.temperature_C = 25.0;
      baro.altitude_m = 0;

      writePacket(file, ID_BARO, baro);
    }

    // GPS Generation: 5 Hz.
    if (static_cast<int>(t * 100) % 20 == 0)
    {
      GpsMeasurement gps;
      gps.timestamp_sec = t;
      gps.sensor_id = 0;

      double home_lat = 32.99;
      double home_lon = -106.97;

      double noise_x = noise_gps_(generator_);
      double noise_y = noise_gps_(generator_);
      double noise_z = noise_gps_(generator_);

      gps.latitude_deg = home_lat + (noise_x / 111132.0);
      gps.longitude_deg = home_lon + (noise_y / (111132.0 * std::cos(home_lat * M_PI / 180.0)));
      gps.altitude_m = pos_z + noise_z;

      gps.num_satellites = 8;
      gps.fix_valid = true;

      writePacket(file, ID_GPS, gps);
    }

    // Magnetometer Generation: 20 Hz.
    if (static_cast<int>(t * 100) % 5 == 0)
    {
      MagMeasurement mag;
      mag.timestamp_sec = t;
      mag.sensor_id = 0;

      double B_north = 20.0;
      double B_east = 0.0;
      double B_down = 40.0;

      double true_mx = B_north;
      double true_my = B_east;
      double true_mz = B_down;

      std::normal_distribution<double> noise_mag(0.0, 0.5);

      mag.mag_x = true_mx + noise_mag(generator_);
      mag.mag_y = true_my + noise_mag(generator_);
      mag.mag_z = true_mz + noise_mag(generator_);

      writePacket(file, ID_MAG, mag);
    }

    // Increment Time.
    t += config_.dt_sec;
  }

  file.close();
  std::cout << "Done! Wrote " << config_.duration_sec << " seconds of data." << std::endl;
}

// Helper to write generic data.
template <typename T>
void DataGenerator::writePacket(std::ofstream &file, uint8_t packet_type, const T &data)
{
  // Write the 1-byte Header.
  file.write(reinterpret_cast<const char *>(&packet_type), sizeof(packet_type));

  // Write the Struct Data.
  file.write(reinterpret_cast<const char *>(&data), sizeof(T));
}