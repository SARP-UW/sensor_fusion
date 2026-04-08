#pragma once

#include <Eigen/Dense>
#include "SensorData.hpp"
#include "FlightPhase.hpp"

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

  Eigen::Vector3d getGyroBias() const;
  Eigen::Vector3d getAccelBias() const;

  FlightPhase getPhase() const;

  Eigen::MatrixXd getCovariance() const { return P_; }

private:
  static const int NOM_DIM = 16;
  static const int ERR_DIM = 15;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  double last_timestamp_sec_;
  bool initialized_;

  bool origin_set_;
  double lat_origin_;
  double lon_origin_;
  double alt_origin_;

  FlightPhaseEstimator phase_estimator_;

  void updateWithMeasurement(const Eigen::VectorXd &innovation,
                             const Eigen::MatrixXd &H,
                             const Eigen::MatrixXd &R);
  Eigen::MatrixXd getProcessNoise(FlightPhase phase) const;

  void applyCorrection(const Eigen::VectorXd &dx);
};