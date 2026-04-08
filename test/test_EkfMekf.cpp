#include "gtest/gtest.h"
#include "Ekf.hpp"
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
  BaroMeasurement b;
  b.timestamp_sec = t;
  b.sensor_id = 0;
  b.pressure_Pa = 101325.0 * std::pow(1.0 - alt_m / 44330.0, 5.255);
  b.temperature_C = 25.0;
  return b;
}

static GpsMeasurement makeGps(double t, double lat, double lon, double alt)
{
  GpsMeasurement g;
  g.timestamp_sec = t;
  g.sensor_id = 0;
  g.latitude_deg = lat;
  g.longitude_deg = lon;
  g.altitude_m = alt;
  g.num_satellites = 8;
  g.fix_valid = true;
  return g;
}

static MagMeasurement makeMag(double t, double yaw_deg)
{
  MagMeasurement m;
  m.timestamp_sec = t;
  m.sensor_id = 0;
  double r = yaw_deg * M_PI / 180.0;
  m.mag_x = 20.0 * std::cos(r);
  m.mag_y = 20.0 * std::sin(r);
  m.mag_z = 40.0;
  return m;
}

TEST(MekfTest, CovarianceIs15x15)
{
  Ekf filter;
  EXPECT_EQ(filter.getCovariance().rows(), 15);
  EXPECT_EQ(filter.getCovariance().cols(), 15);
}

TEST(MekfTest, CovarianceRemains15x15AfterPredict)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  for (int i = 1; i <= 50; ++i)
    filter.predict(makeImu(i * 0.01));
  EXPECT_EQ(filter.getCovariance().rows(), 15);
  EXPECT_EQ(filter.getCovariance().cols(), 15);
}

TEST(MekfTest, CovarianceRemains15x15AfterUpdates)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.01));
  filter.updateBaro(makeBaro(0.01, 0.0));
  filter.updateGps(makeGps(0.01, 32.99, -106.97, 1400.0));
  filter.updateGps(makeGps(0.02, 32.99, -106.97, 1400.0));
  filter.updateMag(makeMag(0.02, 0.0));
  EXPECT_EQ(filter.getCovariance().rows(), 15);
  EXPECT_EQ(filter.getCovariance().cols(), 15);
}

TEST(MekfTest, QuaternionNormIsExactlyOneAfterBaroUpdate)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.01));
  filter.updateBaro(makeBaro(0.01, 100.0));
  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-12);
}

TEST(MekfTest, QuaternionNormIsExactlyOneAfterGpsUpdate)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.01));
  filter.updateGps(makeGps(0.01, 32.99, -106.97, 1400.0));
  filter.predict(makeImu(0.02));
  filter.updateGps(makeGps(0.02, 32.9901, -106.97, 1450.0));
  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-12);
}

TEST(MekfTest, QuaternionNormIsExactlyOneAfterMagUpdate)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.01));
  filter.updateMag(makeMag(0.01, 90.0));
  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-12);
}

TEST(MekfTest, QuaternionNormIsExactlyOneAfterLargeYawCorrection)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.01));
  filter.updateMag(makeMag(0.01, 170.0));
  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-12);
}

TEST(MekfTest, QuaternionNormHoldsAfter500StepsWithContinuousRotation)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 19.81, 0.1, 0.07, 0.05));

  for (int i = 1; i <= 500; ++i)
  {
    filter.predict(makeImu(i * 0.01, 0, 0, 19.81, 0.1, 0.07, 0.05));
    if (i % 10 == 0)
      filter.updateBaro(makeBaro(i * 0.01, 5.0 * i * 0.01));
    if (i % 10 == 0)
      filter.updateMag(makeMag(i * 0.01, 0.0));

    EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-11)
        << "Quaternion norm drifted at step " << i;
  }
}

TEST(MekfTest, AttitudeCovarianceGrowsDuringPredict)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  double p_att_before = filter.getCovariance().block<3, 3>(6, 6).trace();

  for (int i = 1; i <= 50; ++i)
    filter.predict(makeImu(i * 0.01));
  double p_att_after = filter.getCovariance().block<3, 3>(6, 6).trace();

  EXPECT_GT(p_att_after, p_att_before)
      << "Attitude covariance did not grow after predict steps.";
}

TEST(MekfTest, YawCovarianceDecreasesAfterMagUpdate)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  for (int i = 1; i <= 5; ++i)
    filter.predict(makeImu(i * 0.01));

  double p_yaw_before = filter.getCovariance()(8, 8);
  filter.updateMag(makeMag(0.05, 0.0));
  double p_yaw_after = filter.getCovariance()(8, 8);

  EXPECT_LT(p_yaw_after, p_yaw_before)
      << "Yaw variance did not decrease after mag update.";
}

TEST(MekfTest, SequentialMagUpdatesDriveYawTowardMeasurement)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.01));

  double prev_yaw = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    filter.predict(makeImu(0.01 + i * 0.01));
    filter.updateMag(makeMag(0.01 + i * 0.01, 45.0));

    Eigen::Quaterniond q = filter.getOrientation();
    double y2 = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double x2 = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(y2, x2) * 180.0 / M_PI;

    EXPECT_GT(yaw, prev_yaw - 1.0)
        << "Yaw moved away from target at step " << i
        << " (yaw=" << yaw << ", prev=" << prev_yaw << ")";
    prev_yaw = yaw;
  }
  EXPECT_GT(prev_yaw, 5.0)
      << "Yaw did not move meaningfully toward 45° after 20 updates.";
}

TEST(MekfTest, FullFlightCovarianceRemainsHealthy)
{
  const double kHomeLat = 32.99, kHomeLon = -106.97, kHomeAlt = 1400.0;
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81));
  filter.updateGps(makeGps(0.01, kHomeLat, kHomeLon, kHomeAlt));

  for (int i = 1; i <= 500; ++i)
  {
    double t = i * 0.01;
    filter.predict(makeImu(t, 0, 0, 19.81, 0.02, 0.01, 0.03));
    if (i % 10 == 0)
      filter.updateBaro(makeBaro(t, 0.5 * 10.0 * t * t));
    if (i % 20 == 0)
      filter.updateGps(makeGps(t, kHomeLat, kHomeLon, kHomeAlt + 0.5 * 10.0 * t * t));
    if (i % 5 == 0)
      filter.updateMag(makeMag(t, 0.0));

    if (i % 100 == 0)
    {
      Eigen::MatrixXd P = filter.getCovariance();
      ASSERT_EQ(P.rows(), 15) << "P dimension changed at step " << i;
      EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-8)
          << "P became asymmetric at step " << i;

      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> ev(P);
      EXPECT_GE(ev.eigenvalues().minCoeff(), -1e-9)
          << "P has negative eigenvalue at step " << i;

      EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-12)
          << "Quaternion norm drifted at step " << i;

      EXPECT_TRUE(filter.getPosition().allFinite()) << "Position NaN at step " << i;
      EXPECT_TRUE(filter.getVelocity().allFinite()) << "Velocity NaN at step " << i;
      EXPECT_TRUE(filter.getGyroBias().allFinite()) << "GyroBias NaN at step " << i;
      EXPECT_TRUE(filter.getAccelBias().allFinite()) << "AccelBias NaN at step " << i;
    }
  }
}