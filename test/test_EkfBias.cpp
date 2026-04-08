#include "gtest/gtest.h"
#include "Ekf.hpp"
#include "FlightPhase.hpp"
#include <Eigen/Dense>
#include <cmath>

static ImuMeasurement makeImu(double t,
                              double ax = 0.0, double ay = 0.0, double az = 9.81,
                              double gx = 0.0, double gy = 0.0, double gz = 0.0)
{
  ImuMeasurement imu;
  imu.timestamp_sec = t;
  imu.sensor_id = 0;
  imu.acc_x = ax;
  imu.acc_y = ay;
  imu.acc_z = az;
  imu.gyro_x = gx;
  imu.gyro_y = gy;
  imu.gyro_z = gz;
  return imu;
}

static BaroMeasurement makeBaro(double t, double alt_m)
{
  BaroMeasurement baro;
  baro.timestamp_sec = t;
  baro.sensor_id = 0;
  baro.pressure_Pa = 101325.0 * std::pow(1.0 - alt_m / 44330.0, 5.255);
  baro.temperature_C = 25.0;
  return baro;
}

static GpsMeasurement makeGps(double t, double lat, double lon, double alt)
{
  GpsMeasurement gps;
  gps.timestamp_sec = t;
  gps.sensor_id = 0;
  gps.latitude_deg = lat;
  gps.longitude_deg = lon;
  gps.altitude_m = alt;
  gps.num_satellites = 8;
  gps.fix_valid = true;
  return gps;
}

TEST(EkfBiasTest, GyroBiasInitialisesToZero)
{
  Ekf filter;
  EXPECT_NEAR(filter.getGyroBias().x(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getGyroBias().y(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getGyroBias().z(), 0.0, 1e-12);
}

TEST(EkfBiasTest, AccelBiasInitialisesToZero)
{
  Ekf filter;
  EXPECT_NEAR(filter.getAccelBias().x(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getAccelBias().y(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getAccelBias().z(), 0.0, 1e-12);
}

TEST(EkfBiasTest, BiasCovarianceBlocksArePositiveDefinite)
{
  Ekf filter;
  Eigen::MatrixXd P = filter.getCovariance();

  Eigen::Matrix3d P_bg = P.block<3, 3>(9, 9);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> s_bg(P_bg);
  EXPECT_GT(s_bg.eigenvalues().minCoeff(), 0.0)
      << "Gyro bias covariance block is not positive definite.";

  Eigen::Matrix3d P_ba = P.block<3, 3>(12, 12);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> s_ba(P_ba);
  EXPECT_GT(s_ba.eigenvalues().minCoeff(), 0.0)
      << "Accel bias covariance block is not positive definite.";
}

TEST(EkfBiasTest, BiasEstimatesRemainFiniteAfterManyPredicts)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 19.81, 0.01, 0.02, 0.03));

  for (int i = 1; i <= 1000; ++i)
  {
    filter.predict(makeImu(i * 0.01, 0, 0, 19.81, 0.01, 0.02, 0.03));
  }

  EXPECT_TRUE(filter.getGyroBias().allFinite());
  EXPECT_TRUE(filter.getAccelBias().allFinite());
}

TEST(EkfBiasTest, BiasEstimatesDoNotExplodeWithNoBiasInjected)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  for (int i = 1; i <= 500; ++i)
    filter.predict(makeImu(i * 0.01));

  EXPECT_LT(filter.getGyroBias().norm(), 1.0)
      << "Gyro bias drifted unexpectedly far without any injected bias.";
  EXPECT_LT(filter.getAccelBias().norm(), 2.0)
      << "Accel bias drifted unexpectedly far without any injected bias.";
}

TEST(EkfBiasTest, AccelBiasZMovesPosInDirectionOfInjectedBias)
{
  const double injected_bias_z = 1.0;

  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81 + injected_bias_z));

  for (int i = 1; i <= 200; ++i)
  {
    double t = i * 0.01;
    filter.predict(makeImu(t, 0, 0, 9.81 + injected_bias_z));
    filter.updateBaro(makeBaro(t, 0.0));
  }

  EXPECT_GT(filter.getAccelBias().z(), 0.0)
      << "Accel bias z did not move in the correct direction.";
}

TEST(EkfBiasTest, NegativeAccelBiasMovesEstimateNegative)
{
  const double injected_bias_z = -1.0;

  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81 + injected_bias_z));

  for (int i = 1; i <= 200; ++i)
  {
    double t = i * 0.01;
    filter.predict(makeImu(t, 0, 0, 9.81 + injected_bias_z));
    filter.updateBaro(makeBaro(t, 0.0));
  }

  EXPECT_LT(filter.getAccelBias().z(), 0.0)
      << "Accel bias z did not move in the correct direction for negative bias.";
}

TEST(EkfBiasTest, BiasSubtractionReducesAltitudeDriftVsNaiveIntegration)
{
  const double bias_z = 2.0;
  const int warmup = 300;
  const int drift = 100;

  Ekf filterA, filterB;
  filterA.predict(makeImu(0.0, 0, 0, 9.81 + bias_z));
  filterB.predict(makeImu(0.0, 0, 0, 9.81 + bias_z));

  for (int i = 1; i <= warmup; ++i)
  {
    double t = i * 0.01;
    ImuMeasurement imu = makeImu(t, 0, 0, 9.81 + bias_z);
    filterA.predict(imu);
    filterB.predict(imu);
    filterA.updateBaro(makeBaro(t, 0.0));
  }

  for (int i = warmup + 1; i <= warmup + drift; ++i)
  {
    double t = i * 0.01;
    ImuMeasurement imu = makeImu(t, 0, 0, 9.81 + bias_z);
    filterA.predict(imu);
    filterB.predict(imu);
  }

  double errA = std::abs(filterA.getPosition().z());
  double errB = std::abs(filterB.getPosition().z());

  EXPECT_LT(errA, errB)
      << "Filter with baro (bias-observable) drifted MORE than predict-only filter. "
      << "errA=" << errA << " errB=" << errB;
}

TEST(EkfBiasTest, GyroBiasZMovesPosInDirectionOfInjectedBias)
{
  const double injected_bg_z = 0.2;
  const double horizontal_acc = 5.0;
  const double kHomeLat = 32.990000;
  const double kHomeLon = -106.970000;
  const double kHomeAlt = 1400.0;

  Ekf filter;
  filter.predict(makeImu(0.0, horizontal_acc, 0, 29.81, 0, 0, injected_bg_z));
  filter.updateGps(makeGps(0.01, kHomeLat, kHomeLon, kHomeAlt));

  for (int i = 1; i <= 500; ++i)
  {
    double t = i * 0.01;
    filter.predict(makeImu(t, horizontal_acc, 0, 29.81, 0, 0, injected_bg_z));

    if (i % 20 == 0)
    {
      double true_pos_x = 0.5 * horizontal_acc * t * t;

      const double m_per_deg = 111132.0;
      double current_lat = kHomeLat + (true_pos_x / m_per_deg);

      filter.updateGps(makeGps(t, current_lat, kHomeLon, kHomeAlt));
    }
  }

  EXPECT_GT(filter.getGyroBias().z(), 0.0)
      << "Gyro bias z did not move in the correct direction.\n"
      << "bg_z = " << filter.getGyroBias().z() << "\n"
      << "This test requires horizontal acceleration for bg_z to be observable.";
}

TEST(EkfBiasTest, FullCovarianceRemainsSymmetricAndPSDAfterUpdates)
{
  Ekf filter;
  const double kHomeLat = 32.99, kHomeLon = -106.97, kHomeAlt = 1400.0;
  filter.predict(makeImu(0.0, 0, 0, 19.81));
  filter.updateGps(makeGps(0.0, kHomeLat, kHomeLon, kHomeAlt));

  for (int i = 1; i <= 200; ++i)
  {
    double t = i * 0.01;
    filter.predict(makeImu(t, 0, 0, 19.81, 0.01, 0.005, 0.02));
    if (i % 10 == 0)
      filter.updateBaro(makeBaro(t, 5.0 * t * t * 0.01));
    if (i % 20 == 0)
      filter.updateGps(makeGps(t, kHomeLat, kHomeLon, kHomeAlt));
  }

  Eigen::MatrixXd P = filter.getCovariance();
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-8)
      << "Covariance became asymmetric after bias-tracking updates.";

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9)
      << "Covariance has a negative eigenvalue after bias-tracking updates.";
}

TEST(EkfBiasTest, FilterTransitionsToBoostDuringHighAcceleration)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81));

  for (int i = 1; i <= 10; ++i)
    filter.predict(makeImu(i * 0.01, 0, 0, 29.81));

  EXPECT_EQ(filter.getPhase(), FlightPhase::BOOST)
      << "Filter did not transition to BOOST after sustained high acceleration.";
}

TEST(EkfBiasTest, FilterRemainsPreLaunchDuringIdleAcceleration)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  for (int i = 1; i <= 100; ++i)
    filter.predict(makeImu(i * 0.01));

  EXPECT_EQ(filter.getPhase(), FlightPhase::PRE_LAUNCH)
      << "Filter incorrectly left PRE_LAUNCH during idle.";
}