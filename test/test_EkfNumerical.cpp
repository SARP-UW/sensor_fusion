#include "gtest/gtest.h"
#include "Ekf.hpp"
#include "DataGenerator.hpp"
#include "DataIngestion.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <string>

static void expectSymmetric(const Eigen::MatrixXd &P, const std::string &label)
{
  double asymmetry = (P - P.transpose()).norm();
  EXPECT_NEAR(asymmetry, 0.0, 1e-8) << label << ": covariance is not symmetric.";
}

static void expectPSD(const Eigen::MatrixXd &P, const std::string &label)
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
  ASSERT_EQ(solver.info(), Eigen::Success) << label << ": eigendecomposition failed.";
  double min_ev = solver.eigenvalues().minCoeff();
  EXPECT_GE(min_ev, -1e-9)
      << label << ": covariance has negative eigenvalue " << min_ev
      << " — filter has gone indefinite.";
}

static void expectQuatNorm(const Ekf &filter, const std::string &label)
{
  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-7)
      << label << ": quaternion is no longer unit norm.";
}

TEST(EkfNumericalTest, CovarianceRemainsSymmetricAndPSDAfter100Predicts)
{
  Ekf filter;

  ImuMeasurement imu;
  imu.sensor_id = 0;
  imu.acc_z = 19.81;
  imu.gyro_x = 0.05;
  imu.gyro_y = 0.03;
  imu.gyro_z = 0.07;
  imu.timestamp_sec = 0.0;
  filter.predict(imu);

  for (int i = 1; i <= 100; ++i)
  {
    imu.timestamp_sec = i * 0.01;
    filter.predict(imu);

    if (i % 10 == 0)
    {
      std::string tag = "after " + std::to_string(i) + " predicts";
      expectSymmetric(filter.getCovariance(), tag);
      expectPSD(filter.getCovariance(), tag);
      expectQuatNorm(filter, tag);
    }
  }
}

TEST(EkfNumericalTest, CovarianceRemainsHealthyAfterInterleavedUpdates)
{
  Ekf filter;
  const double kHomeLat = 32.99;
  const double kHomeLon = -106.97;
  const double kHomeAlt = 1400.0;

  {
    ImuMeasurement imu;
    imu.timestamp_sec = 0.0;
    imu.sensor_id = 0;
    imu.acc_z = 9.81;
    filter.predict(imu);
  }

  {
    GpsMeasurement gps;
    gps.timestamp_sec = 0.01;
    gps.sensor_id = 0;
    gps.latitude_deg = kHomeLat;
    gps.longitude_deg = kHomeLon;
    gps.altitude_m = kHomeAlt;
    gps.num_satellites = 8;
    gps.fix_valid = true;
    filter.updateGps(gps);
  }

  for (int i = 1; i <= 200; ++i)
  {
    double t = i * 0.01;

    {
      ImuMeasurement imu;
      imu.timestamp_sec = t;
      imu.sensor_id = 0;
      imu.acc_z = 19.81;
      imu.gyro_x = 0.01;
      imu.gyro_y = 0.005;
      imu.gyro_z = 0.02;
      filter.predict(imu);
    }

    if (i % 10 == 0)
    {
      BaroMeasurement baro;
      baro.timestamp_sec = t;
      baro.sensor_id = 0;
      double simulated_alt = 0.5 * 10.0 * t * t;
      baro.pressure_Pa = 101325.0 * std::pow(1.0 - simulated_alt / 44330.0, 5.255);
      baro.temperature_C = 25.0;
      filter.updateBaro(baro);
    }

    if (i % 20 == 0)
    {
      GpsMeasurement gps;
      gps.timestamp_sec = t;
      gps.sensor_id = 0;
      double simulated_alt = 0.5 * 10.0 * t * t;
      gps.latitude_deg = kHomeLat;
      gps.longitude_deg = kHomeLon;
      gps.altitude_m = kHomeAlt + simulated_alt;
      gps.num_satellites = 8;
      gps.fix_valid = true;
      filter.updateGps(gps);
    }

    if (i % 5 == 0)
    {
      MagMeasurement mag;
      mag.timestamp_sec = t;
      mag.sensor_id = 0;
      mag.mag_x = 20.0;
      mag.mag_y = 0.0;
      mag.mag_z = 40.0;
      filter.updateMag(mag);
    }

    if (i % 50 == 0)
    {
      std::string tag = "step " + std::to_string(i);
      expectSymmetric(filter.getCovariance(), tag);
      expectPSD(filter.getCovariance(), tag);
      expectQuatNorm(filter, tag);
    }
  }
}

TEST(EkfNumericalTest, AltitudeConvergesWithRepeatedBaroUpdates)
{
  Ekf filter;

  ImuMeasurement imu;
  imu.timestamp_sec = 0.0;
  imu.sensor_id = 0;
  imu.acc_z = 9.81;
  filter.predict(imu);

  const double target_alt = 100.0;
  for (int i = 1; i <= 200; ++i)
  {
    double t = i * 0.1;
    imu.timestamp_sec = t;
    filter.predict(imu);

    BaroMeasurement baro;
    baro.timestamp_sec = t;
    baro.sensor_id = 0;
    baro.pressure_Pa = 101325.0 * std::pow(1.0 - target_alt / 44330.0, 5.255);
    baro.temperature_C = 25.0;
    filter.updateBaro(baro);
  }

  EXPECT_NEAR(filter.getPosition().z(), target_alt, 5.0)
      << "Altitude did not converge to target after 200 baro updates.";
}

class EkfEndToEndTest : public ::testing::Test
{
protected:
  static const std::string kFile;
  static void SetUpTestSuite()
  {
    DataGenerator::Config cfg;
    cfg.duration_sec = 30.0;
    cfg.output_filename = kFile;
    DataGenerator gen(cfg);
    gen.generate();
  }
  static void TearDownTestSuite()
  {
    std::remove(kFile.c_str());
  }
};

const std::string EkfEndToEndTest::kFile = "test_e2e_flight.bin";

TEST_F(EkfEndToEndTest, FilterDoesNotProduceNaNOrInfDuringFullFlight)
{
  DataIngestion parser;
  ParsedData data = parser.loadFromFile(kFile);
  ASSERT_FALSE(data.imu_data.empty());

  Ekf filter;
  bool gps_origin_set = false;

  size_t imu_i = 0, baro_i = 0, gps_i = 0, mag_i = 0;

  for (size_t step = 0; step < data.imu_data.size(); ++step)
  {
    filter.predict(data.imu_data[step]);

    if (baro_i < data.baro_data.size() && step % 10 == 0)
      filter.updateBaro(data.baro_data[baro_i++]);

    if (gps_i < data.gps_data.size() && step % 20 == 0)
    {
      filter.updateGps(data.gps_data[gps_i++]);
      gps_origin_set = true;
    }

    if (mag_i < data.mag_data.size() && step % 5 == 0)
      filter.updateMag(data.mag_data[mag_i++]);
  }

  EXPECT_TRUE(filter.getPosition().allFinite())
      << "Position contains NaN or Inf after full flight.";
  EXPECT_TRUE(filter.getVelocity().allFinite())
      << "Velocity contains NaN or Inf after full flight.";

  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_TRUE(std::isfinite(q.w()) && std::isfinite(q.x()) &&
              std::isfinite(q.y()) && std::isfinite(q.z()))
      << "Quaternion contains NaN or Inf after full flight.";
  EXPECT_NEAR(q.norm(), 1.0, 1e-6)
      << "Quaternion lost unit norm after full flight.";

  Eigen::MatrixXd P = filter.getCovariance();
  EXPECT_TRUE(P.allFinite())
      << "Covariance contains NaN or Inf after full flight.";
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-8)
      << "Covariance is not symmetric after full flight.";

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9)
      << "Covariance has negative eigenvalue after full flight.";
}

TEST_F(EkfEndToEndTest, AltitudeIsPositiveAfterBurnPhase)
{
  DataIngestion parser;
  ParsedData data = parser.loadFromFile(kFile);
  ASSERT_FALSE(data.imu_data.empty());

  Ekf filter;
  size_t baro_i = 0, gps_i = 0, mag_i = 0;

  for (size_t step = 0; step < data.imu_data.size(); ++step)
  {
    filter.predict(data.imu_data[step]);
    if (baro_i < data.baro_data.size() && step % 10 == 0)
      filter.updateBaro(data.baro_data[baro_i++]);
    if (gps_i < data.gps_data.size() && step % 20 == 0)
      filter.updateGps(data.gps_data[gps_i++]);
    if (mag_i < data.mag_data.size() && step % 5 == 0)
      filter.updateMag(data.mag_data[mag_i++]);

    if (step == 1000)
    {
      EXPECT_GT(filter.getPosition().z(), 50.0)
          << "Altitude implausibly low after burn phase.";
    }
  }
}