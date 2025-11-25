#pragma once

#include <vector>
#include <cstdint>

// Base Measurement: All sensors share these common traits.
struct SensorMeasurement
{
  // Monotonic timestamp in seconds.
  double timestamp_sec;

  // ID to distinguish redundant sensors.
  uint8_t sensor_id;
};

// Inertial Measurement Unit (IMU).
struct ImuMeasurement : public SensorMeasurement
{
  // Accelerations in m/s^2.
  double acc_x;
  double acc_y;
  double acc_z;

  // Angular Rates in rad/s.
  double gyro_x;
  double gyro_y;
  double gyro_z;
};

// Barometer.
struct BaroMeasurement : public SensorMeasurement
{
  // Pressure in Pascals.
  double pressure_Pa;

  // Altitude in meters.
  double altitude_m;

  // Internal Temperature in Celsius.
  double temperature_C;
};

// GPS / GNSS.
struct GpsMeasurement : public SensorMeasurement
{
  // Latitude and Longitude in decimal degrees.
  double latitude_deg;
  double longitude_deg;

  // Altitude in meters.
  double altitude_m;

  // Ground speed in m/s.
  double ground_speed_ms;

  // Number of satellites in view.
  uint8_t num_satellites;

  // Fix validity.
  bool fix_valid;
};

// Magnetometer.
struct MagMeasurement : public SensorMeasurement
{
  // Magnetic field strength in micro-Tesla (uT).
  double mag_x;
  double mag_y;
  double mag_z;
};

// External Temperature.
struct TempMeasurement : public SensorMeasurement
{
  // Temperature in Celsius.
  double temperature_C;
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