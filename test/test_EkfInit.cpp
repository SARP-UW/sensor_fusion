#include "gtest/gtest.h"
#include "Ekf.hpp"
#include <Eigen/Dense>
#include <cmath>

TEST(EkfInitTest, PositionIsZero)
{
  Ekf filter;
  Eigen::Vector3d pos = filter.getPosition();
  EXPECT_NEAR(pos.x(), 0.0, 1e-12);
  EXPECT_NEAR(pos.y(), 0.0, 1e-12);
  EXPECT_NEAR(pos.z(), 0.0, 1e-12);
}

TEST(EkfInitTest, VelocityIsZero)
{
  Ekf filter;
  Eigen::Vector3d vel = filter.getVelocity();
  EXPECT_NEAR(vel.x(), 0.0, 1e-12);
  EXPECT_NEAR(vel.y(), 0.0, 1e-12);
  EXPECT_NEAR(vel.z(), 0.0, 1e-12);
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
  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_NEAR(q.norm(), 1.0, 1e-12);
}

TEST(EkfInitTest, CovarianceIsSymmetric)
{
  Ekf filter;
  Eigen::MatrixXd P = filter.getCovariance();
  double asymmetry = (P - P.transpose()).norm();
  EXPECT_NEAR(asymmetry, 0.0, 1e-12) << "Initial covariance P is not symmetric.";
}

TEST(EkfInitTest, CovarianceIsPositiveDefinite)
{
  Ekf filter;
  Eigen::MatrixXd P = filter.getCovariance();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
  ASSERT_EQ(solver.info(), Eigen::Success);
  double min_eigenvalue = solver.eigenvalues().minCoeff();
  EXPECT_GT(min_eigenvalue, 0.0)
      << "Initial covariance P has a non-positive eigenvalue: " << min_eigenvalue;
}

TEST(EkfInitTest, CovarianceHasCorrectDimension)
{
  Ekf filter;
  Eigen::MatrixXd P = filter.getCovariance();
  EXPECT_EQ(P.rows(), 10);
  EXPECT_EQ(P.cols(), 10);
}

TEST(EkfInitTest, AllStateComponentsAreFinite)
{
  Ekf filter;
  Eigen::Vector3d pos = filter.getPosition();
  Eigen::Vector3d vel = filter.getVelocity();
  Eigen::Quaterniond q = filter.getOrientation();

  EXPECT_TRUE(pos.allFinite());
  EXPECT_TRUE(vel.allFinite());
  EXPECT_TRUE(std::isfinite(q.w()));
  EXPECT_TRUE(std::isfinite(q.x()));
  EXPECT_TRUE(std::isfinite(q.y()));
  EXPECT_TRUE(std::isfinite(q.z()));
}