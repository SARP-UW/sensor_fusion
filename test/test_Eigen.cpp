#include "gtest/gtest.h"
#include <Eigen/Dense>

TEST(EigenTest, ShouldPerformMatrixMultiplication)
{
    Eigen::Matrix2d A;
    A << 1, 2,
        3, 4;

    Eigen::Vector2d B;
    B << 1,
        1;

    Eigen::Vector2d x = A * B;

    EXPECT_EQ(x(0), 3.0);
    EXPECT_EQ(x(1), 7.0);
}