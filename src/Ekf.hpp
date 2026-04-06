#pragma once

#include <Eigen/Dense>
#include "SensorData.hpp"

class Ekf
{
public:
  Ekf();

  void predict(const ImuMeasurement &imu);

  void updateBaro(const BaroMeasurement &baro);
  void updateGps(const GpsMeasurement &gps);
  void updateMag(const MagMeasurement &mag);

  Eigen::Vector3d getPosition() const;
  Eigen::Vector3d getVelocity() const;
  Eigen::Quaterniond getOrientation() const;

  Eigen::MatrixXd getCovariance() const { return P_; }

private:
  static const int STATE_DIM = 10;

  Eigen::VectorXd x_;

  Eigen::MatrixXd P_;

  double last_timestamp_sec_;

  bool initialized_;

  bool origin_set_;
  double lat_origin_;
  double lon_origin_;
  double alt_origin_;
};