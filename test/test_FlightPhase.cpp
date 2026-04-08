#include "gtest/gtest.h"
#include "FlightPhase.hpp"
#include <string>

static void advance(FlightPhaseEstimator &est, int steps,
                    double alt, double vel_z, double accel, double dt = 0.01)
{
  for (int i = 0; i < steps; ++i)
    est.update(alt, vel_z, accel, dt);
}

TEST(FlightPhaseStringTest, AllPhasesHaveNonEmptyString)
{
  EXPECT_FALSE(flightPhaseToString(FlightPhase::PRE_LAUNCH).empty());
  EXPECT_FALSE(flightPhaseToString(FlightPhase::BOOST).empty());
  EXPECT_FALSE(flightPhaseToString(FlightPhase::COAST).empty());
  EXPECT_FALSE(flightPhaseToString(FlightPhase::DESCENT).empty());
  EXPECT_FALSE(flightPhaseToString(FlightPhase::LANDED).empty());
}

TEST(FlightPhaseStringTest, StringsAreDistinct)
{
  EXPECT_NE(flightPhaseToString(FlightPhase::PRE_LAUNCH),
            flightPhaseToString(FlightPhase::BOOST));
  EXPECT_NE(flightPhaseToString(FlightPhase::BOOST),
            flightPhaseToString(FlightPhase::COAST));
  EXPECT_NE(flightPhaseToString(FlightPhase::COAST),
            flightPhaseToString(FlightPhase::DESCENT));
  EXPECT_NE(flightPhaseToString(FlightPhase::DESCENT),
            flightPhaseToString(FlightPhase::LANDED));
}

TEST(FlightPhaseEstimatorTest, DefaultConstructorStartsAtPreLaunch)
{
  FlightPhaseEstimator est;
  EXPECT_EQ(est.getPhase(), FlightPhase::PRE_LAUNCH);
}

TEST(FlightPhaseEstimatorTest, TimeInPhaseStartsAtZero)
{
  FlightPhaseEstimator est;
  EXPECT_NEAR(est.getTimeInPhase(), 0.0, 1e-12);
}

TEST(FlightPhaseEstimatorTest, CustomConfigConstructorStartsAtPreLaunch)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_accel_threshold_ms2 = 20.0;
  FlightPhaseEstimator est(cfg);
  EXPECT_EQ(est.getPhase(), FlightPhase::PRE_LAUNCH);
}

TEST(FlightPhaseEstimatorTest, StaysInPreLaunchBelowThreshold)
{
  FlightPhaseEstimator est;
  advance(est, 100, 0.0, 0.0, 14.9);
  EXPECT_EQ(est.getPhase(), FlightPhase::PRE_LAUNCH);
}

TEST(FlightPhaseEstimatorTest, TransitionsToBoostAfterConfirmationSamples)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 5;
  cfg.launch_accel_threshold_ms2 = 15.0;
  FlightPhaseEstimator est(cfg);

  advance(est, 4, 0.0, 0.0, 20.0);
  EXPECT_EQ(est.getPhase(), FlightPhase::PRE_LAUNCH);

  est.update(0.0, 0.0, 20.0, 0.01);
  EXPECT_EQ(est.getPhase(), FlightPhase::BOOST);
}

TEST(FlightPhaseEstimatorTest, LaunchCounterResetsOnSubThresholdSample)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 5;
  cfg.launch_accel_threshold_ms2 = 15.0;
  FlightPhaseEstimator est(cfg);

  advance(est, 4, 0.0, 0.0, 20.0);
  est.update(0.0, 0.0, 10.0, 0.01);
  advance(est, 4, 0.0, 0.0, 20.0);
  EXPECT_EQ(est.getPhase(), FlightPhase::PRE_LAUNCH);
}

TEST(FlightPhaseEstimatorTest, TimeInPhaseResetsOnTransition)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 5;
  cfg.launch_accel_threshold_ms2 = 15.0;
  FlightPhaseEstimator est(cfg);

  advance(est, 10, 0.0, 0.0, 5.0);
  advance(est, 5, 0.0, 0.0, 20.0);

  EXPECT_NEAR(est.getTimeInPhase(), 0.0, 1e-9)
      << "TimeInPhase should reset to 0 on transition.";
}

TEST(FlightPhaseEstimatorTest, TransitionsToCoastOnBurnout)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.burnout_accel_threshold_ms2 = 12.0;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::BOOST);

  est.update(100.0, 50.0, 5.0, 0.01);
  EXPECT_EQ(est.getPhase(), FlightPhase::COAST);
}

TEST(FlightPhaseEstimatorTest, TransitionsToCoastOnBurnTimeTimeout)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.max_burn_time_s = 0.1;
  cfg.burnout_accel_threshold_ms2 = 5.0;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::BOOST);

  advance(est, 11, 50.0, 50.0, 20.0, 0.01);
  EXPECT_EQ(est.getPhase(), FlightPhase::COAST);
}

TEST(FlightPhaseEstimatorTest, TransitionsToDescentAtApogee)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.burnout_accel_threshold_ms2 = 12.0;
  cfg.min_altitude_for_apogee_m = 10.0;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  est.update(50.0, 50.0, 5.0, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::COAST);

  est.update(200.0, -1.0, 9.81, 0.01);
  EXPECT_EQ(est.getPhase(), FlightPhase::DESCENT);
}

TEST(FlightPhaseEstimatorTest, DoesNotTransitionToDescentBelowMinAltitude)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.burnout_accel_threshold_ms2 = 12.0;
  cfg.min_altitude_for_apogee_m = 100.0;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  est.update(50.0, 50.0, 5.0, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::COAST);

  est.update(5.0, -1.0, 9.81, 0.01);
  EXPECT_EQ(est.getPhase(), FlightPhase::COAST);
}

TEST(FlightPhaseEstimatorTest, TransitionsToLandedAfterConfirmationSamples)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.burnout_accel_threshold_ms2 = 12.0;
  cfg.min_altitude_for_apogee_m = 10.0;
  cfg.landed_alt_threshold_m = 2.0;
  cfg.landed_vel_threshold_ms = 1.0;
  cfg.landed_confirm_samples = 3;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  est.update(50.0, 50.0, 5.0, 0.01);
  est.update(200.0, -1.0, 9.81, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::DESCENT);

  advance(est, 2, 1.0, 0.1, 9.81);
  EXPECT_EQ(est.getPhase(), FlightPhase::DESCENT);

  est.update(1.0, 0.1, 9.81, 0.01);
  EXPECT_EQ(est.getPhase(), FlightPhase::LANDED);
}

TEST(FlightPhaseEstimatorTest, LandedCounterResetsWhenConditionsNotMet)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.burnout_accel_threshold_ms2 = 12.0;
  cfg.min_altitude_for_apogee_m = 10.0;
  cfg.landed_alt_threshold_m = 2.0;
  cfg.landed_vel_threshold_ms = 1.0;
  cfg.landed_confirm_samples = 5;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  est.update(50.0, 50.0, 5.0, 0.01);
  est.update(200.0, -1.0, 9.81, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::DESCENT);

  advance(est, 3, 1.0, 0.1, 9.81);
  est.update(1.0, 5.0, 9.81, 0.01);
  advance(est, 4, 1.0, 0.1, 9.81);
  EXPECT_EQ(est.getPhase(), FlightPhase::DESCENT);
}

TEST(FlightPhaseEstimatorTest, LandedIsTerminalState)
{
  FlightPhaseEstimator::Config cfg;
  cfg.launch_confirm_samples = 1;
  cfg.launch_accel_threshold_ms2 = 15.0;
  cfg.burnout_accel_threshold_ms2 = 12.0;
  cfg.min_altitude_for_apogee_m = 10.0;
  cfg.landed_alt_threshold_m = 2.0;
  cfg.landed_vel_threshold_ms = 1.0;
  cfg.landed_confirm_samples = 1;
  FlightPhaseEstimator est(cfg);

  est.update(0.0, 10.0, 20.0, 0.01);
  est.update(50.0, 50.0, 5.0, 0.01);
  est.update(200.0, -1.0, 9.81, 0.01);
  est.update(1.0, 0.1, 9.81, 0.01);
  ASSERT_EQ(est.getPhase(), FlightPhase::LANDED);

  advance(est, 100, 500.0, 200.0, 50.0);
  EXPECT_EQ(est.getPhase(), FlightPhase::LANDED);
}

TEST(FlightPhaseEstimatorTest, GetPhaseStringMatchesToString)
{
  FlightPhaseEstimator est;
  EXPECT_EQ(est.getPhaseString(), flightPhaseToString(FlightPhase::PRE_LAUNCH));
}