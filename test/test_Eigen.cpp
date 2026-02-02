#include "gtest/gtest.h"
#include <Eigen/Dense>

TEST(EigenTest, ShouldPerformMatrixMultiplication)
{
  // Define a 2x2 Matrix A.
  Eigen::Matrix2d A;
  A << 1, 2,
      3, 4;

  // Define a 2x1 Vector B.
  Eigen::Vector2d B;
  B << 1,
      1;

  // Perform Multiplication: x = A * B.
  Eigen::Vector2d x = A * B;

  EXPECT_EQ(x(0), 3.0);
  EXPECT_EQ(x(1), 7.0);
}