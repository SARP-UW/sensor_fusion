#include "FlightPhase.hpp"
#include <cmath>

std::string flightPhaseToString(FlightPhase phase)
{
  switch (phase)
  {
  case FlightPhase::PRE_LAUNCH:
    return "PRE_LAUNCH";
  case FlightPhase::BOOST:
    return "BOOST";
  case FlightPhase::COAST:
    return "COAST";
  case FlightPhase::DESCENT:
    return "DESCENT";
  case FlightPhase::LANDED:
    return "LANDED";
  default:
    return "UNKNOWN";
  }
}

FlightPhaseEstimator::FlightPhaseEstimator(Config cfg) : cfg_(cfg) {}

void FlightPhaseEstimator::update(double alt_m, double vel_z_ms,
                                  double imu_accel_norm_ms2, double dt_s)
{
  time_in_phase_s_ += dt_s;

  switch (phase_)
  {
  case FlightPhase::PRE_LAUNCH:
  {
    if (imu_accel_norm_ms2 > cfg_.launch_accel_threshold_ms2)
    {
      ++launch_counter_;
      if (launch_counter_ >= cfg_.launch_confirm_samples)
      {
        phase_ = FlightPhase::BOOST;
        time_in_phase_s_ = 0.0;
        launch_counter_ = 0;
      }
    }
    else
    {
      launch_counter_ = 0;
    }
    break;
  }

  case FlightPhase::BOOST:
  {
    bool burnout_detected = imu_accel_norm_ms2 < cfg_.burnout_accel_threshold_ms2;
    bool timeout = time_in_phase_s_ > cfg_.max_burn_time_s;

    if (burnout_detected || timeout)
    {
      phase_ = FlightPhase::COAST;
      time_in_phase_s_ = 0.0;
    }
    break;
  }

  case FlightPhase::COAST:
  {
    if (vel_z_ms < 0.0 && alt_m > cfg_.min_altitude_for_apogee_m)
    {
      phase_ = FlightPhase::DESCENT;
      time_in_phase_s_ = 0.0;
    }
    break;
  }

  case FlightPhase::DESCENT:
  {
    bool near_ground = alt_m < cfg_.landed_alt_threshold_m;
    bool slow = std::abs(vel_z_ms) < cfg_.landed_vel_threshold_ms;

    if (near_ground && slow)
    {
      ++landed_counter_;
      if (landed_counter_ >= cfg_.landed_confirm_samples)
      {
        phase_ = FlightPhase::LANDED;
        time_in_phase_s_ = 0.0;
      }
    }
    else
    {
      landed_counter_ = 0;
    }
    break;
  }

  case FlightPhase::LANDED:
    break;
  }
}