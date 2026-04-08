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

TEST(EkfPredictTest, FirstCallOnlyInitializesTimestamp)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81));

  EXPECT_NEAR(filter.getPosition().norm(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getVelocity().norm(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getOrientation().w(), 1.0, 1e-12);
}

TEST(EkfPredictTest, ZeroDtDoesNotChangeState)
{
  Ekf filter;
  filter.predict(makeImu(0.0));
  filter.predict(makeImu(0.0));

  EXPECT_NEAR(filter.getPosition().norm(), 0.0, 1e-12);
  EXPECT_NEAR(filter.getVelocity().norm(), 0.0, 1e-12);
}

TEST(EkfPredictTest, NegativeDtDoesNotChangeState)
{
  Ekf filter;
  filter.predict(makeImu(1.0));
  Eigen::Vector3d pos_before = filter.getPosition();

  filter.predict(makeImu(0.5));
  EXPECT_NEAR((filter.getPosition() - pos_before).norm(), 0.0, 1e-12);
}

TEST(EkfPredictTest, LargeDtIsClamped)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81));
  filter.predict(makeImu(100.0, 0, 0, 9.81));

  Eigen::Vector3d vel = filter.getVelocity();
  EXPECT_LT(std::abs(vel.z()), 2.0)
      << "dt was not clamped — velocity blew up.";
}

TEST(EkfPredictTest, GravityOnlyAccelProducesZeroNetAcceleration)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81));
  for (int i = 1; i <= 100; ++i)
  {
    filter.predict(makeImu(i * 0.01, 0, 0, 9.81));
  }

  EXPECT_NEAR(filter.getVelocity().norm(), 0.0, 1e-6);
  EXPECT_NEAR(filter.getPosition().norm(), 0.0, 1e-6);
}

TEST(EkfPredictTest, VerticalAccelerationIntegratesCorrectlyVelocity)
{
  Ekf filter;
  const double dt = 0.01;
  const double sim_az = 19.81;
  filter.predict(makeImu(0.0, 0, 0, sim_az));

  for (int i = 1; i <= 100; ++i)
  {
    filter.predict(makeImu(i * dt, 0, 0, sim_az));
  }

  EXPECT_NEAR(filter.getVelocity().z(), 10.0, 0.05);
}

TEST(EkfPredictTest, VerticalAccelerationIntegratesCorrectlyPosition)
{
  Ekf filter;
  const double dt = 0.01;
  const double sim_az = 19.81;
  filter.predict(makeImu(0.0, 0, 0, sim_az));

  for (int i = 1; i <= 100; ++i)
  {
    filter.predict(makeImu(i * dt, 0, 0, sim_az));
  }

  EXPECT_NEAR(filter.getPosition().z(), 5.0, 0.05);
}

TEST(EkfPredictTest, HorizontalAccelerationIntegratesIntoXY)
{
  Ekf filter;
  const double dt = 0.01;
  filter.predict(makeImu(0.0, 1.0, 0, 9.81));

  for (int i = 1; i <= 100; ++i)
  {
    filter.predict(makeImu(i * dt, 1.0, 0, 9.81));
  }

  EXPECT_NEAR(filter.getVelocity().x(), 1.0, 0.05);
  EXPECT_NEAR(filter.getPosition().x(), 0.5, 0.05);
}

TEST(EkfPredictTest, ZeroGyroDoesNotRotateQuaternion)
{
  Ekf filter;
  filter.predict(makeImu(0.0));

  for (int i = 1; i <= 100; ++i)
  {
    filter.predict(makeImu(i * 0.01));
  }

  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_NEAR(q.w(), 1.0, 1e-6);
  EXPECT_NEAR(q.x(), 0.0, 1e-6);
  EXPECT_NEAR(q.y(), 0.0, 1e-6);
  EXPECT_NEAR(q.z(), 0.0, 1e-6);
}

TEST(EkfPredictTest, GyroRateRotatesQuaternion)
{
  Ekf filter;
  const double gz = M_PI / 2.0;
  const double dt = 0.001;
  filter.predict(makeImu(0.0, 0, 0, 9.81, 0, 0, gz));

  for (int i = 1; i <= 1000; ++i)
  {
    filter.predict(makeImu(i * dt, 0, 0, 9.81, 0, 0, gz));
  }

  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_NEAR(q.norm(), 1.0, 1e-6);
  EXPECT_NEAR(q.z(), 0.707, 0.02);
  EXPECT_NEAR(q.w(), 0.707, 0.02);
}

TEST(EkfPredictTest, CovarianceGrowsAfterPredict)
{
  Ekf filter;
  double p_diag_initial = filter.getCovariance().diagonal().sum();

  filter.predict(makeImu(0.0));

  for (int i = 1; i <= 50; ++i)
  {
    filter.predict(makeImu(i * 0.01));
  }

  double p_diag_final = filter.getCovariance().diagonal().sum();
  EXPECT_GT(p_diag_final, p_diag_initial)
      << "Covariance did not grow after predict steps — process noise may be zero.";
}

TEST(EkfPredictTest, CovarianceRemainsSymmetricAfterPredict)
{
  Ekf filter;
  filter.predict(makeImu(0.0));

  for (int i = 1; i <= 50; ++i)
  {
    filter.predict(makeImu(i * 0.01));
  }

  Eigen::MatrixXd P = filter.getCovariance();
  double asymmetry = (P - P.transpose()).norm();
  EXPECT_NEAR(asymmetry, 0.0, 1e-10)
      << "Covariance became asymmetric after predict steps.";
}

TEST(EkfPredictTest, QuaternionRemainsNormalizedAfterManyPredicts)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 9.81, 0.05, 0.03, 0.07));

  for (int i = 1; i <= 500; ++i)
  {
    filter.predict(makeImu(i * 0.01, 0, 0, 9.81, 0.05, 0.03, 0.07));
  }

  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-9);
}

TEST(EkfPredictTest, AllStatesRemainFiniteAfterManyPredicts)
{
  Ekf filter;
  filter.predict(makeImu(0.0, 0, 0, 19.81, 0.01, 0.02, 0.03));

  for (int i = 1; i <= 1000; ++i)
  {
    filter.predict(makeImu(i * 0.01, 0, 0, 19.81, 0.01, 0.02, 0.03));
  }

  EXPECT_TRUE(filter.getPosition().allFinite());
  EXPECT_TRUE(filter.getVelocity().allFinite());
  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_TRUE(std::isfinite(q.w()) && std::isfinite(q.x()) &&
              std::isfinite(q.y()) && std::isfinite(q.z()));
}