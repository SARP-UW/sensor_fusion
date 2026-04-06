#include "DataGenerator.hpp"
#include <fstream>
#include <iostream>
#include <cmath>

const uint8_t ID_IMU = 0x10;
const uint8_t ID_BARO = 0x20;
const uint8_t ID_GPS = 0x30;
const uint8_t ID_MAG = 0x40;

DataGenerator::DataGenerator(Config config) : config_(config) {}

void DataGenerator::generate()
{
  std::ofstream file(config_.output_filename, std::ios::binary);
  if (!file.is_open())
  {
    std::cerr << "Failed to open output file!" << std::endl;
    return;
  }
  std::cout << "Generating synthetic flight data..." << std::endl;

  const int total_steps = static_cast<int>(config_.duration_sec / config_.dt_sec);
  const int imu_rate_hz = static_cast<int>(std::round(1.0 / config_.dt_sec));
  const int baro_every = imu_rate_hz / 10;
  const int gps_every = imu_rate_hz / 5;
  const int mag_every = imu_rate_hz / 20;

  double pos_z = 0.0;
  double vel_z = 0.0;
  double true_acc_z = 0.0;

  const double burn_time = 10.0;

  for (int step = 0; step < total_steps; ++step)
  {
    const double t = step * config_.dt_sec;

    if (t < burn_time)
    {
      true_acc_z = 20.0;
    }
    else if (vel_z > 0)
    {
      true_acc_z = -9.81;
    }
    else if (pos_z > 0)
    {
      true_acc_z = 0.0;
      vel_z = -5.0;
    }
    else
    {
      true_acc_z = 0.0;
      vel_z = 0.0;
      pos_z = 0.0;
    }

    if (pos_z > 0 || t < burn_time)
    {
      vel_z += true_acc_z * config_.dt_sec;
      pos_z += vel_z * config_.dt_sec;
    }

    {
      ImuMeasurement imu;
      imu.timestamp_sec = t;
      imu.sensor_id = 0;
      imu.acc_x = 0.0 + noise_imu_acc_(generator_);
      imu.acc_y = 0.0 + noise_imu_acc_(generator_);
      imu.acc_z = true_acc_z + 9.81 + noise_imu_acc_(generator_);
      imu.gyro_x = noise_imu_gyro_(generator_);
      imu.gyro_y = noise_imu_gyro_(generator_);
      imu.gyro_z = noise_imu_gyro_(generator_);
      writePacket(file, ID_IMU, imu);
    }

    if (step % baro_every == 0)
    {
      BaroMeasurement baro;
      baro.timestamp_sec = t;
      baro.sensor_id = 0;
      const double sea_level_pa = 101325.0;
      const double true_pressure = sea_level_pa * std::pow(1.0 - (pos_z / 44330.0), 5.255);
      baro.pressure_Pa = true_pressure + noise_baro_(generator_);
      baro.temperature_C = 25.0;
      writePacket(file, ID_BARO, baro);
    }

    if (step % gps_every == 0)
    {
      GpsMeasurement gps;
      gps.timestamp_sec = t;
      gps.sensor_id = 0;
      const double home_lat = 32.99;
      const double home_lon = -106.97;
      const double nx = noise_gps_(generator_);
      const double ny = noise_gps_(generator_);
      const double nz = noise_gps_(generator_);
      gps.latitude_deg = home_lat + (nx / 111132.0);
      gps.longitude_deg = home_lon + (ny / (111132.0 * std::cos(home_lat * M_PI / 180.0)));
      gps.altitude_m = pos_z + nz;
      gps.num_satellites = 8;
      gps.fix_valid = true;
      writePacket(file, ID_GPS, gps);
    }

    if (step % mag_every == 0)
    {
      MagMeasurement mag;
      mag.timestamp_sec = t;
      mag.sensor_id = 0;
      std::normal_distribution<double> noise_mag(0.0, 0.5);
      mag.mag_x = 20.0 + noise_mag(generator_);
      mag.mag_y = 0.0 + noise_mag(generator_);
      mag.mag_z = 40.0 + noise_mag(generator_);
      writePacket(file, ID_MAG, mag);
    }
  }

  file.close();
  std::cout << "Done! Wrote " << config_.duration_sec << " seconds of data." << std::endl;
}

template <typename T>
void DataGenerator::writePacket(std::ofstream &file, uint8_t packet_type, const T &data)
{
  file.write(reinterpret_cast<const char *>(&packet_type), sizeof(packet_type));
  file.write(reinterpret_cast<const char *>(&data), sizeof(T));
}