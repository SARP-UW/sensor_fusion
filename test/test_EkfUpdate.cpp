#include "gtest/gtest.h"
#include "Ekf.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

static ImuMeasurement makeImu(double t, double az = 9.81,
                              double gx = 0.0, double gy = 0.0, double gz = 0.0)
{
  ImuMeasurement imu;
  imu.timestamp_sec = t;
  imu.sensor_id = 0;
  imu.acc_x = 0.0;
  imu.acc_y = 0.0;
  imu.acc_z = az;
  imu.gyro_x = gx;
  imu.gyro_y = gy;
  imu.gyro_z = gz;
  return imu;
}

static double altToPressure(double alt_m)
{
  return 101325.0 * std::pow(1.0 - alt_m / 44330.0, 5.255);
}

static BaroMeasurement makeBaro(double t, double alt_m)
{
  BaroMeasurement baro;
  baro.timestamp_sec = t;
  baro.sensor_id = 0;
  baro.pressure_Pa = altToPressure(alt_m);
  baro.temperature_C = 25.0;
  return baro;
}

static GpsMeasurement makeGps(double t, double lat, double lon,
                              double alt, bool fix = true)
{
  GpsMeasurement gps;
  gps.timestamp_sec = t;
  gps.sensor_id = 0;
  gps.latitude_deg = lat;
  gps.longitude_deg = lon;
  gps.altitude_m = alt;
  gps.num_satellites = 8;
  gps.fix_valid = fix;
  return gps;
}

static MagMeasurement makeMag(double t, double yaw_deg)
{
  MagMeasurement mag;
  mag.timestamp_sec = t;
  mag.sensor_id = 0;
  double yaw_rad = yaw_deg * M_PI / 180.0;
  mag.mag_x = 20.0 * std::cos(yaw_rad);
  mag.mag_y = 20.0 * std::sin(yaw_rad);
  mag.mag_z = 40.0;
  return mag;
}

class EkfBaroTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    filter_.predict(makeImu(0.0));
    filter_.predict(makeImu(0.1));
  }
  Ekf filter_;
};

TEST_F(EkfBaroTest, BaroMovesPredictedAltitudeTowardMeasurement)
{
  double before = filter_.getPosition().z();
  filter_.updateBaro(makeBaro(0.1, 100.0));
  double after = filter_.getPosition().z();

  EXPECT_GT(after, before) << "Baro update did not move altitude upward.";
  EXPECT_LT(after, 105.0) << "Baro overcorrected past the measurement.";
}

TEST_F(EkfBaroTest, BaroDoesNotOvercorrect)
{
  filter_.updateBaro(makeBaro(0.1, 100.0));
  EXPECT_LT(filter_.getPosition().z(), 100.0)
      << "Filter fully replaced its estimate with the measurement — no blending.";
}

TEST_F(EkfBaroTest, RepeatedBaroUpdatesMovesEstimateTowardMeasurement)
{
  const double target = 200.0;
  for (int i = 0; i < 100; ++i)
  {
    filter_.predict(makeImu(0.1 + i * 0.1));
    filter_.updateBaro(makeBaro(0.1 + i * 0.1, target));
  }
  double z = filter_.getPosition().z();
  EXPECT_GT(z, 20.0) << "Estimate barely moved after 100 baro updates.";
  EXPECT_LT(z, 600.0) << "Estimate diverged — filter is unstable.";
}

TEST_F(EkfBaroTest, BaroEstimateSettlesNearTargetFromSmallOffset)
{
  const double target = 20.0;
  for (int i = 0; i < 300; ++i)
  {
    filter_.predict(makeImu(0.1 + i * 0.01));
    filter_.updateBaro(makeBaro(0.1 + i * 0.01, target));
  }
  EXPECT_NEAR(filter_.getPosition().z(), target, 10.0)
      << "Altitude did not settle near target after 300-step warm-up.";
}

TEST_F(EkfBaroTest, RejectsZeroPressure)
{
  BaroMeasurement bad = makeBaro(0.1, 100.0);
  bad.pressure_Pa = 0.0;
  double z_before = filter_.getPosition().z();
  filter_.updateBaro(bad);
  EXPECT_NEAR(filter_.getPosition().z(), z_before, 1e-12)
      << "Filter accepted a zero-pressure measurement.";
}

TEST_F(EkfBaroTest, RejectsNegativePressure)
{
  BaroMeasurement bad = makeBaro(0.1, 100.0);
  bad.pressure_Pa = -1000.0;
  double z_before = filter_.getPosition().z();
  filter_.updateBaro(bad);
  EXPECT_NEAR(filter_.getPosition().z(), z_before, 1e-12)
      << "Filter accepted a negative-pressure measurement.";
}

TEST_F(EkfBaroTest, RejectsNaNPressure)
{
  BaroMeasurement bad = makeBaro(0.1, 100.0);
  bad.pressure_Pa = std::numeric_limits<double>::quiet_NaN();
  double z_before = filter_.getPosition().z();
  filter_.updateBaro(bad);
  EXPECT_NEAR(filter_.getPosition().z(), z_before, 1e-12)
      << "Filter accepted a NaN pressure measurement.";
}

TEST_F(EkfBaroTest, QuaternionRemainsNormalizedAfterUpdate)
{
  filter_.updateBaro(makeBaro(0.1, 100.0));
  EXPECT_NEAR(filter_.getOrientation().norm(), 1.0, 1e-9);
}

TEST_F(EkfBaroTest, AltitudeCovarianceDecreasesAfterUpdate)
{
  double p_zz_before = filter_.getCovariance()(2, 2);
  filter_.updateBaro(makeBaro(0.1, 100.0));
  double p_zz_after = filter_.getCovariance()(2, 2);

  EXPECT_LT(p_zz_after, p_zz_before)
      << "Altitude covariance did not decrease after a baro update.";
}

TEST_F(EkfBaroTest, CovarianceRemainsSymmetricAfterUpdate)
{
  filter_.updateBaro(makeBaro(0.1, 100.0));
  Eigen::MatrixXd P = filter_.getCovariance();
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-10);
}

class EkfGpsTest : public ::testing::Test
{
protected:
  static constexpr double kHomeLat = 32.990000;
  static constexpr double kHomeLon = -106.970000;
  static constexpr double kHomeAlt = 1400.0;

  void SetUp() override
  {
    filter_.predict(makeImu(0.0));
    filter_.predict(makeImu(0.1));

    filter_.updateGps(makeGps(0.1, kHomeLat, kHomeLon, kHomeAlt));
  }
  Ekf filter_;
};

TEST_F(EkfGpsTest, FirstGpsCallSetsOriginWithoutChangingState)
{
  EXPECT_NEAR(filter_.getPosition().norm(), 0.0, 1e-6);
}

TEST_F(EkfGpsTest, GpsMovesPredictedPositionTowardMeasurement)
{
  double lat_north = kHomeLat + (50.0 / 111132.0);
  filter_.updateGps(makeGps(0.2, lat_north, kHomeLon, kHomeAlt + 50.0));

  EXPECT_GT(filter_.getPosition().z(), 0.0) << "GPS z correction failed.";
  EXPECT_GT(filter_.getPosition().x(), 0.0) << "GPS x correction failed.";
}

TEST_F(EkfGpsTest, RejectsInvalidFix)
{
  GpsMeasurement bad = makeGps(0.2, kHomeLat, kHomeLon, kHomeAlt + 50.0, false);
  Eigen::Vector3d pos_before = filter_.getPosition();
  filter_.updateGps(bad);
  EXPECT_NEAR((filter_.getPosition() - pos_before).norm(), 0.0, 1e-12)
      << "Filter accepted a GPS packet with fix_valid=false.";
}

TEST_F(EkfGpsTest, RejectsNaNLatitude)
{
  GpsMeasurement bad = makeGps(0.2, kHomeLat, kHomeLon, kHomeAlt);
  bad.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d pos_before = filter_.getPosition();
  filter_.updateGps(bad);
  EXPECT_NEAR((filter_.getPosition() - pos_before).norm(), 0.0, 1e-12)
      << "Filter accepted a GPS packet with NaN latitude.";
}

TEST_F(EkfGpsTest, RejectsNaNLongitude)
{
  GpsMeasurement bad = makeGps(0.2, kHomeLat, kHomeLon, kHomeAlt);
  bad.longitude_deg = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d pos_before = filter_.getPosition();
  filter_.updateGps(bad);
  EXPECT_NEAR((filter_.getPosition() - pos_before).norm(), 0.0, 1e-12)
      << "Filter accepted a GPS packet with NaN longitude.";
}

TEST_F(EkfGpsTest, QuaternionRemainsNormalizedAfterUpdate)
{
  filter_.updateGps(makeGps(0.2, kHomeLat, kHomeLon, kHomeAlt + 50.0));
  EXPECT_NEAR(filter_.getOrientation().norm(), 1.0, 1e-9);
}

TEST_F(EkfGpsTest, PositionCovarianceDecreasesAfterUpdate)
{
  double p_xx_before = filter_.getCovariance()(0, 0);
  filter_.updateGps(makeGps(0.2, kHomeLat, kHomeLon, kHomeAlt + 50.0));
  double p_xx_after = filter_.getCovariance()(0, 0);

  EXPECT_LT(p_xx_after, p_xx_before)
      << "X-position covariance did not decrease after GPS update.";
}

TEST_F(EkfGpsTest, CovarianceRemainsSymmetricAfterUpdate)
{
  filter_.updateGps(makeGps(0.2, kHomeLat, kHomeLon, kHomeAlt + 50.0));
  Eigen::MatrixXd P = filter_.getCovariance();
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-10);
}

class EkfMagTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    filter_.predict(makeImu(0.0));
    filter_.predict(makeImu(0.1));
  }
  Ekf filter_;
};

TEST_F(EkfMagTest, NorthPointingMagLeavesYawNearZero)
{
  filter_.updateMag(makeMag(0.1, 0.0));

  Eigen::Quaterniond q = filter_.getOrientation();
  EXPECT_NEAR(q.w(), 1.0, 0.01);
  EXPECT_NEAR(q.z(), 0.0, 0.01);
}

TEST_F(EkfMagTest, EastPointingMagRotatesYawPositive)
{
  filter_.updateMag(makeMag(0.1, 90.0));

  EXPECT_GT(filter_.getOrientation().z(), 0.001)
      << "Filter did not rotate toward 90° yaw.";
}

TEST_F(EkfMagTest, QuaternionRemainsUnitNormAfterUpdate)
{
  filter_.updateMag(makeMag(0.1, 45.0));
  EXPECT_NEAR(filter_.getOrientation().norm(), 1.0, 1e-9);
}

TEST_F(EkfMagTest, AllComponentsFiniteAfterUpdate)
{
  filter_.updateMag(makeMag(0.1, 135.0));
  Eigen::Quaterniond q = filter_.getOrientation();
  EXPECT_TRUE(std::isfinite(q.w()) && std::isfinite(q.x()) &&
              std::isfinite(q.y()) && std::isfinite(q.z()));
}

TEST_F(EkfMagTest, RejectsNaNMagX)
{
  MagMeasurement bad = makeMag(0.1, 0.0);
  bad.mag_x = std::numeric_limits<double>::quiet_NaN();
  Eigen::Quaterniond q_before = filter_.getOrientation();
  filter_.updateMag(bad);
  EXPECT_NEAR(filter_.getOrientation().w(), q_before.w(), 1e-12);
}

TEST_F(EkfMagTest, CovarianceRemainsSymmetricAfterUpdate)
{
  filter_.updateMag(makeMag(0.1, 45.0));
  Eigen::MatrixXd P = filter_.getCovariance();
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-10);
}

TEST_F(EkfMagTest, RepeatedNorthMagDoesNotDriftYaw)
{
  for (int i = 0; i < 50; ++i)
  {
    filter_.predict(makeImu(0.1 + i * 0.05));
    filter_.updateMag(makeMag(0.1 + i * 0.05, 0.0));
  }

  Eigen::Quaterniond q = filter_.getOrientation();
  double yaw_deg = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())) *
                   180.0 / M_PI;
  EXPECT_NEAR(yaw_deg, 0.0, 5.0)
      << "Yaw drifted under repeated north-pointing mag updates.";
}