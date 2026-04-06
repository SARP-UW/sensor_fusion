#include "gtest/gtest.h"
#include "DataGenerator.hpp"
#include "DataIngestion.hpp"
#include <cstdio>
#include <cmath>
#include <string>

class DataGeneratorTest : public ::testing::Test
{
protected:
  static const std::string kFile;
  static ParsedData data_;

  static void SetUpTestSuite()
  {
    DataGenerator::Config cfg;
    cfg.duration_sec = 30.0;
    cfg.output_filename = kFile;
    DataGenerator gen(cfg);
    gen.generate();

    DataIngestion parser;
    data_ = parser.loadFromFile(kFile);
  }

  static void TearDownTestSuite()
  {
    std::remove(kFile.c_str());
  }
};

const std::string DataGeneratorTest::kFile = "test_gen_flight.bin";
ParsedData DataGeneratorTest::data_ = {};

TEST_F(DataGeneratorTest, ImuPacketCountIsCorrect)
{
  EXPECT_EQ(data_.imu_data.size(), 3000u);
}

TEST_F(DataGeneratorTest, BaroPacketCountIsCorrect)
{
  EXPECT_EQ(data_.baro_data.size(), 300u);
}

TEST_F(DataGeneratorTest, GpsPacketCountIsCorrect)
{
  EXPECT_EQ(data_.gps_data.size(), 150u);
}

TEST_F(DataGeneratorTest, MagPacketCountIsCorrect)
{
  EXPECT_EQ(data_.mag_data.size(), 600u);
}

TEST_F(DataGeneratorTest, ImuTimestampsAreMonotonicallyNonDecreasing)
{
  for (size_t i = 1; i < data_.imu_data.size(); ++i)
  {
    EXPECT_GE(data_.imu_data[i].timestamp_sec,
              data_.imu_data[i - 1].timestamp_sec)
        << "Timestamp regression at IMU packet " << i;
  }
}

TEST_F(DataGeneratorTest, ImuTimestampsStartAtZero)
{
  ASSERT_FALSE(data_.imu_data.empty());
  EXPECT_NEAR(data_.imu_data.front().timestamp_sec, 0.0, 1e-9);
}

TEST_F(DataGeneratorTest, ImuTimestepIsUniform)
{
  for (size_t i = 1; i < data_.imu_data.size(); ++i)
  {
    double dt = data_.imu_data[i].timestamp_sec - data_.imu_data[i - 1].timestamp_sec;
    EXPECT_NEAR(dt, 0.01, 1e-9)
        << "Non-uniform timestep at IMU packet " << i;
  }
}

TEST_F(DataGeneratorTest, BaroTimestampsAreMonotonicallyNonDecreasing)
{
  for (size_t i = 1; i < data_.baro_data.size(); ++i)
  {
    EXPECT_GE(data_.baro_data[i].timestamp_sec,
              data_.baro_data[i - 1].timestamp_sec)
        << "Timestamp regression at baro packet " << i;
  }
}

TEST_F(DataGeneratorTest, ImuAccZDuringBurnIncludesGravity)
{
  const double expected = 29.81;
  const double tol = 3.0;

  for (const auto &imu : data_.imu_data)
  {
    if (imu.timestamp_sec < 9.0)
    {
      EXPECT_NEAR(imu.acc_z, expected, tol)
          << "acc_z out of expected burn range at t=" << imu.timestamp_sec;
    }
  }
}

TEST_F(DataGeneratorTest, ImuAccZDuringCoastIsNearZero)
{
  bool found_coast = false;
  for (const auto &imu : data_.imu_data)
  {
    if (imu.timestamp_sec >= 12.0 && imu.timestamp_sec <= 28.0)
    {
      EXPECT_NEAR(imu.acc_z, 0.0, 1.5)
          << "acc_z out of expected coast range at t=" << imu.timestamp_sec;
      found_coast = true;
    }
  }
  EXPECT_TRUE(found_coast) << "No IMU packets found in coast phase window.";
}

TEST_F(DataGeneratorTest, BaroPressureDecreasesDuringBurn)
{
  double p_start = data_.baro_data.front().pressure_Pa;
  double p_at_burn_end = 0.0;
  for (const auto &b : data_.baro_data)
  {
    if (std::abs(b.timestamp_sec - 10.0) < 0.15)
    {
      p_at_burn_end = b.pressure_Pa;
      break;
    }
  }
  ASSERT_GT(p_at_burn_end, 0.0) << "No baro packet found near t=10s.";
  EXPECT_LT(p_at_burn_end, p_start)
      << "Pressure should decrease as altitude increases.";
}

TEST_F(DataGeneratorTest, GpsFixIsAlwaysValid)
{
  for (const auto &gps : data_.gps_data)
  {
    EXPECT_TRUE(gps.fix_valid)
        << "GPS fix dropped at t=" << gps.timestamp_sec;
  }
}

TEST_F(DataGeneratorTest, GpsSatelliteCountIsNonZero)
{
  for (const auto &gps : data_.gps_data)
  {
    EXPECT_GT(gps.num_satellites, 0)
        << "Zero satellites at t=" << gps.timestamp_sec;
  }
}

TEST_F(DataGeneratorTest, MagFieldMagnitudeIsPhysicallyReasonable)
{
  for (const auto &mag : data_.mag_data)
  {
    double mag_norm = std::sqrt(mag.mag_x * mag.mag_x +
                                mag.mag_y * mag.mag_y +
                                mag.mag_z * mag.mag_z);
    EXPECT_GT(mag_norm, 35.0) << "Mag norm implausibly small.";
    EXPECT_LT(mag_norm, 55.0) << "Mag norm implausibly large.";
  }
}

TEST_F(DataGeneratorTest, SensorIdsAreZero)
{
  for (const auto &imu : data_.imu_data)
    EXPECT_EQ(imu.sensor_id, 0);
  for (const auto &baro : data_.baro_data)
    EXPECT_EQ(baro.sensor_id, 0);
  for (const auto &gps : data_.gps_data)
    EXPECT_EQ(gps.sensor_id, 0);
  for (const auto &mag : data_.mag_data)
    EXPECT_EQ(mag.sensor_id, 0);
}