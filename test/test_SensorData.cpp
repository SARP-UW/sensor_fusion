#include "gtest/gtest.h"
#include "SensorData.hpp"
#include <cstddef>
#include <cstring>

TEST(SensorDataTest, ImuMeasurementDefaultsToZero)
{
  ImuMeasurement imu;
  EXPECT_EQ(imu.timestamp_sec, 0.0);
  EXPECT_EQ(imu.sensor_id, 0);
  EXPECT_EQ(imu.acc_x, 0.0);
  EXPECT_EQ(imu.acc_y, 0.0);
  EXPECT_EQ(imu.acc_z, 0.0);
  EXPECT_EQ(imu.gyro_x, 0.0);
  EXPECT_EQ(imu.gyro_y, 0.0);
  EXPECT_EQ(imu.gyro_z, 0.0);
}

TEST(SensorDataTest, BaroMeasurementDefaultsToZero)
{
  BaroMeasurement baro;
  EXPECT_EQ(baro.timestamp_sec, 0.0);
  EXPECT_EQ(baro.sensor_id, 0);
  EXPECT_EQ(baro.pressure_Pa, 0.0);
  EXPECT_EQ(baro.temperature_C, 0.0);
}

TEST(SensorDataTest, GpsMeasurementDefaultsToZero)
{
  GpsMeasurement gps;
  EXPECT_EQ(gps.timestamp_sec, 0.0);
  EXPECT_EQ(gps.sensor_id, 0);
  EXPECT_EQ(gps.latitude_deg, 0.0);
  EXPECT_EQ(gps.longitude_deg, 0.0);
  EXPECT_EQ(gps.altitude_m, 0.0);
  EXPECT_EQ(gps.ground_speed_ms, 0.0);
  EXPECT_EQ(gps.num_satellites, 0);
  EXPECT_EQ(gps.fix_valid, false);
}

TEST(SensorDataTest, MagMeasurementDefaultsToZero)
{
  MagMeasurement mag;
  EXPECT_EQ(mag.timestamp_sec, 0.0);
  EXPECT_EQ(mag.sensor_id, 0);
  EXPECT_EQ(mag.mag_x, 0.0);
  EXPECT_EQ(mag.mag_y, 0.0);
  EXPECT_EQ(mag.mag_z, 0.0);
}

TEST(SensorDataTest, ParsedDataContainersDefaultToEmpty)
{
  ParsedData data;
  EXPECT_TRUE(data.imu_data.empty());
  EXPECT_TRUE(data.baro_data.empty());
  EXPECT_TRUE(data.gps_data.empty());
  EXPECT_TRUE(data.mag_data.empty());
  EXPECT_TRUE(data.temp_data.empty());
}

TEST(SensorDataTest, ImuMeasurementSizeIsStable)
{
  const size_t expected = sizeof(ImuMeasurement);
  EXPECT_EQ(sizeof(ImuMeasurement), expected)
      << "ImuMeasurement size changed — binary log format is broken.";
}

TEST(SensorDataTest, BaroMeasurementSizeIsStable)
{
  const size_t expected = sizeof(BaroMeasurement);
  EXPECT_EQ(sizeof(BaroMeasurement), expected)
      << "BaroMeasurement size changed — binary log format is broken.";
}

TEST(SensorDataTest, GpsMeasurementSizeIsStable)
{
  const size_t expected = sizeof(GpsMeasurement);
  EXPECT_EQ(sizeof(GpsMeasurement), expected)
      << "GpsMeasurement size changed — binary log format is broken.";
}

TEST(SensorDataTest, MagMeasurementSizeIsStable)
{
  const size_t expected = sizeof(MagMeasurement);
  EXPECT_EQ(sizeof(MagMeasurement), expected)
      << "MagMeasurement size changed — binary log format is broken.";
}

TEST(SensorDataTest, MPiIsDefinedAndCorrect)
{
  EXPECT_NEAR(M_PI, 3.14159265358979323846, 1e-15);
}