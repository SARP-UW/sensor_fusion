#include "gtest/gtest.h"
#include "DataIngestion.hpp"

TEST(SimpleTest, ShouldPass)
{
  EXPECT_TRUE(true);
}

TEST(LinkingTest, ShouldLinkToSrcLibrary)
{
  std::string message = getTestMessage();
  EXPECT_EQ(message, "Library Linked!");
}