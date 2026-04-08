#include "gtest/gtest.h"
#include "Ekf.hpp"
#include "FlightPhase.hpp"
#include <Eigen/Dense>
#include <cmath>

TEST(EkfInitTest, PositionIsZero)
{
  Ekf filter;
  EXPECT_NEAR(filter.getPosition().norm(), 0.0, 1e-12);
}

TEST(EkfInitTest, VelocityIsZero)
{
  Ekf filter;
  EXPECT_NEAR(filter.getVelocity().norm(), 0.0, 1e-12);
}

TEST(EkfInitTest, OrientationIsIdentityQuaternion)
{
  Ekf filter;
  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_NEAR(q.w(), 1.0, 1e-12);
  EXPECT_NEAR(q.x(), 0.0, 1e-12);
  EXPECT_NEAR(q.y(), 0.0, 1e-12);
  EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(EkfInitTest, OrientationIsUnitNorm)
{
  Ekf filter;
  EXPECT_NEAR(filter.getOrientation().norm(), 1.0, 1e-12);
}

TEST(EkfInitTest, GyroBiasIsZero)
{
  Ekf filter;
  EXPECT_NEAR(filter.getGyroBias().norm(), 0.0, 1e-12);
}

TEST(EkfInitTest, AccelBiasIsZero)
{
  Ekf filter;
  EXPECT_NEAR(filter.getAccelBias().norm(), 0.0, 1e-12);
}

TEST(EkfInitTest, PhaseIsPreLaunchAtInit)
{
  Ekf filter;
  EXPECT_EQ(filter.getPhase(), FlightPhase::PRE_LAUNCH);
}

TEST(EkfInitTest, CovarianceHasCorrectDimension)
{
  Ekf filter;
  EXPECT_EQ(filter.getCovariance().rows(), 16);
  EXPECT_EQ(filter.getCovariance().cols(), 16);
}

TEST(EkfInitTest, CovarianceIsSymmetric)
{
  Ekf filter;
  Eigen::MatrixXd P = filter.getCovariance();
  EXPECT_NEAR((P - P.transpose()).norm(), 0.0, 1e-12);
}

TEST(EkfInitTest, CovarianceIsPositiveDefinite)
{
  Ekf filter;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(filter.getCovariance());
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(EkfInitTest, AllStateComponentsAreFinite)
{
  Ekf filter;
  EXPECT_TRUE(filter.getPosition().allFinite());
  EXPECT_TRUE(filter.getVelocity().allFinite());
  EXPECT_TRUE(filter.getGyroBias().allFinite());
  EXPECT_TRUE(filter.getAccelBias().allFinite());
  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_TRUE(std::isfinite(q.w()) && std::isfinite(q.x()) &&
              std::isfinite(q.y()) && std::isfinite(q.z()));
}