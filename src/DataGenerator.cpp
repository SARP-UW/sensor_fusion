#include "DataGenerator.hpp"
#include <fstream>
#include <iostream>
#include <cmath>

// Packet Headers (Magic Bytes) to identify data in the file.
const uint8_t ID_IMU = 0x10;
const uint8_t ID_BARO = 0x20;
const uint8_t ID_GPS = 0x30;

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
  double t = 0.0;
  double pos_z = 0.0;      // Altitude (m).
  double vel_z = 0.0;      // Velocity (m/s).
  double true_acc_z = 0.0; // Kinematic acceleration (m/s^2).

  // Flight Events.
  double burn_time = 5.0;   // Engine burns for 5 seconds.
  double apogee_time = 0.0; // Will be set automatically.

  while (t < config_.duration_sec)
  {
    // Simple Physics Engine.

    if (t < burn_time)
    {
      // Phase 1: Burn (Motor is firing).
      // Rocket accelerates up at ~2 Gs (approx 20 m/s^2).
      true_acc_z = 20.0;
    }
    else if (vel_z > 0)
    {
      // Phase 2: Coast (Motor off, gravity slows us down).
      // Gravity is -9.81.
      true_acc_z = -9.81;
    }
    else if (pos_z > 0)
    {
      // Phase 3: Descent (Parachute deployed).
      // Fall at steady terminal velocity of -5 m/s.
      // Acceleration is roughly 0 (forces balanced).
      true_acc_z = 0.0;
      vel_z = -5.0;
    }
    else
    {
      // Landed.
      true_acc_z = 0.0;
      vel_z = 0.0;
      pos_z = 0.0;
    }

    // Integrate Physics (Euler Integration).
    if (pos_z > 0 || t < burn_time)
    {
      vel_z += true_acc_z * config_.dt_sec;
      pos_z += vel_z * config_.dt_sec;
    }

    // Generate Sensor Measurements.

    // IMU Generation (100 Hz - Runs every loop).
    // Accelerometer sees (True Accel + Gravity).
    // Example: On pad, true_acc=0, but sensor feels 9.81 pushing up.
    double sensed_acc_z = true_acc_z + 9.81;

    ImuMeasurement imu;
    imu.timestamp_sec = t;
    imu.sensor_id = 0;
    imu.acc_x = 0.0 + noise_imu_acc_(generator_);
    imu.acc_y = 0.0 + noise_imu_acc_(generator_);
    imu.acc_z = sensed_acc_z + noise_imu_acc_(generator_);
    imu.gyro_x = 0.0 + noise_imu_gyro_(generator_);
    imu.gyro_y = 0.0 + noise_imu_gyro_(generator_);
    imu.gyro_z = 0.0 + noise_imu_gyro_(generator_);

    writePacket(file, ID_IMU, imu);

    // Barometer Generation (Every 0.1s approx).
    // Simple modulo check to simulate 10Hz sample rate.
    if (static_cast<int>(t * 100) % 10 == 0)
    {
      BaroMeasurement baro;
      baro.timestamp_sec = t;
      baro.sensor_id = 0;

      // Simplified Barometric Formula: P = P0 * (1 - L*h/T0)^(gM/RL).
      double sea_level_pa = 101325.0;

      // Rough approximation near sea level.
      double true_pressure = sea_level_pa - (pos_z * 12.0);

      baro.pressure_Pa = true_pressure + noise_baro_(generator_);
      baro.temperature_C = 25.0;
      baro.altitude_m = 0;

      writePacket(file, ID_BARO, baro);
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