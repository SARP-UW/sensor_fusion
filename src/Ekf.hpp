#pragma once

#include <Eigen/Dense>
#include "SensorData.hpp"

class Ekf
{
public:
  // Constructor.
  Ekf();

  // Uses IMU data to predict the next state.
  void predict(const ImuMeasurement &imu);

  // Uses sensor data to correct the state.
  void updateBaro(const BaroMeasurement &baro);
  void updateGps(const GpsMeasurement &gps);
  void updateMag(const MagMeasurement &mag);

  // Getters.
  Eigen::Vector3d getPosition() const;
  Eigen::Vector3d getVelocity() const;
  Eigen::Quaterniond getOrientation() const;

private:
  // 10 state variables (3 Pos, 3 Vel, 4 Quat).
  static const int STATE_DIM = 10;

  // The State Vector 'x'.
  Eigen::VectorXd x_;

  // The Covariance Matrix 'P'.
  Eigen::MatrixXd P_;

  // Timekeeping.
  double last_timestamp_sec_;

  // Initialization flag.
  bool initialized_;

  // GPS Origin.
  bool origin_set_;
  double lat_origin_;
  double lon_origin_;
  double alt_origin_;
};