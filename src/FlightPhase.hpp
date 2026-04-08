#pragma once

#include <string>

enum class FlightPhase
{
  PRE_LAUNCH,
  BOOST,
  COAST,
  DESCENT,
  LANDED
};

std::string flightPhaseToString(FlightPhase phase);

class FlightPhaseEstimator
{
public:
  struct Config
  {
    double launch_accel_threshold_ms2 = 15.0;
    int launch_confirm_samples = 5;

    double burnout_accel_threshold_ms2 = 12.0;
    double max_burn_time_s = 30.0;

    double min_altitude_for_apogee_m = 10.0;

    double landed_alt_threshold_m = 2.0;
    double landed_vel_threshold_ms = 1.0;
    int landed_confirm_samples = 20;
  };

  FlightPhaseEstimator() = default;
  explicit FlightPhaseEstimator(Config cfg);

  void update(double alt_m, double vel_z_ms,
              double imu_accel_norm_ms2, double dt_s);

  FlightPhase getPhase() const { return phase_; }
  double getTimeInPhase() const { return time_in_phase_s_; }
  std::string getPhaseString() const { return flightPhaseToString(phase_); }

private:
  Config cfg_;
  FlightPhase phase_ = FlightPhase::PRE_LAUNCH;
  double time_in_phase_s_ = 0.0;
  int launch_counter_ = 0;
  int landed_counter_ = 0;
};