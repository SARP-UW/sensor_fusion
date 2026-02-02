#pragma once

#include <vector>
#include <cstdint>

// Base Measurement.
struct SensorMeasurement
{
  double timestamp_sec = 0.0;
  uint8_t sensor_id = 0;
};

// Inertial Measurement Unit (IMU).
struct ImuMeasurement : public SensorMeasurement
{
  double acc_x = 0.0; double acc_y = 0.0; double acc_z = 0.0;
  double gyro_x = 0.0; double gyro_y = 0.0; double gyro_z = 0.0;
};

// Barometer.
struct BaroMeasurement : public SensorMeasurement
{
  double pressure_Pa = 0.0;
  double altitude_m = 0.0;
  double temperature_C = 0.0;
};

// GPS / GNSS.
struct GpsMeasurement : public SensorMeasurement
{
  double latitude_deg = 0.0; double longitude_deg = 0.0;
  double altitude_m = 0.0;
  double ground_speed_ms = 0.0;
  uint8_t num_satellites = 0;
  bool fix_valid = false;
};

// Magnetometer.
struct MagMeasurement : public SensorMeasurement
{
  double mag_x = 0.0;
  double mag_y = 0.0;
  double mag_z = 0.0;
};

// External Temperature.
struct TempMeasurement : public SensorMeasurement
{
  double temperature_C = 0.0;
};

// Container for a complete flight log.
struct ParsedData
{
  std::vector<ImuMeasurement> imu_data;
  std::vector<BaroMeasurement> baro_data;
  std::vector<GpsMeasurement> gps_data;
  std::vector<MagMeasurement> mag_data;
  std::vector<TempMeasurement> temp_data;
};