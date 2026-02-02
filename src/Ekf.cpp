#include "Ekf.hpp"
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper to check for NaNs
bool hasNaN(const Eigen::VectorXd &x)
{
  for (int i = 0; i < x.size(); ++i)
  {
    if (std::isnan(x(i)))
      return true;
  }
  return false;
}

Ekf::Ekf()
{
  x_ = Eigen::VectorXd::Zero(STATE_DIM);
  x_(6) = 1.0;

  P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P_.block<6, 6>(0, 0) *= 0.1;
  P_.block<4, 4>(6, 6) *= 0.1;

  last_timestamp_sec_ = 0.0;
  initialized_ = false;
  origin_set_ = false;
  lat_origin_ = 0;
  lon_origin_ = 0;
  alt_origin_ = 0;

  std::cout << "EKF Initialized." << std::endl;
}

void Ekf::predict(const ImuMeasurement &imu)
{
  if (!initialized_)
  {
    last_timestamp_sec_ = imu.timestamp_sec;
    initialized_ = true;
    return;
  }
  double dt = imu.timestamp_sec - last_timestamp_sec_;
  last_timestamp_sec_ = imu.timestamp_sec;
  if (dt <= 0)
    return;

  // --- 1. STATE PREDICTION ---
  Eigen::Vector3d pos = getPosition();
  Eigen::Vector3d vel = getVelocity();
  Eigen::Quaterniond q = getOrientation();

  // Orientation
  Eigen::Vector3d omega(imu.gyro_x, imu.gyro_y, imu.gyro_z);
  Eigen::Quaterniond q_dot;
  q_dot.w() = 0;
  q_dot.vec() = omega;
  Eigen::Vector4d q_new_coeffs = q.coeffs() + (0.5 * dt) * (q * q_dot).coeffs();
  Eigen::Quaterniond q_new;
  q_new.coeffs() = q_new_coeffs;
  q_new.normalize();

  x_(6) = q_new.w();
  x_(7) = q_new.x();
  x_(8) = q_new.y();
  x_(9) = q_new.z();

  // Position/Velocity
  Eigen::Vector3d acc_body(imu.acc_x, imu.acc_y, imu.acc_z);
  Eigen::Vector3d acc_world = q_new * acc_body;
  Eigen::Vector3d gravity(0, 0, 9.81);
  Eigen::Vector3d acc_net = acc_world - gravity;

  Eigen::Vector3d vel_new = vel + (acc_net * dt);
  x_.segment<3>(3) = vel_new;

  Eigen::Vector3d pos_new = pos + (vel * dt) + (0.5 * acc_net * dt * dt);
  x_.segment<3>(0) = pos_new;

  // --- 2. COVARIANCE  ---
  // P = F * P * F^t + Q

  // F (State Transition Matrix) approx
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt; // Pos depends on Vel

  // Q (Process Noise)
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  double q_pos = 0.0001;
  double q_vel = 0.01;
  double q_att = 0.001;

  Q.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * q_pos;
  Q.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * q_vel;
  Q.block<4, 4>(6, 6) = Eigen::MatrixXd::Identity(4, 4) * q_att;

  P_ = (F * P_ * F.transpose()) + Q;
}

void Ekf::updateBaro(const BaroMeasurement &baro)
{
  if (std::isnan(baro.pressure_Pa) || std::isnan(baro.altitude_m))
    return;
  if (baro.pressure_Pa <= 0.0)
    return;

  double P0 = 101325.0;
  double measured_alt = 44330.0 * (1.0 - std::pow(baro.pressure_Pa / P0, 1.0 / 5.255));

  Eigen::MatrixXd R(1, 1);
  R << 4.0;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
  H(0, 2) = 1.0;

  Eigen::VectorXd z(1);
  z << measured_alt;
  Eigen::VectorXd y = z - (H * x_);
  Eigen::MatrixXd S = (H * P_ * H.transpose()) + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P_ = (I - (K * H)) * P_;
}

void Ekf::updateGps(const GpsMeasurement &gps)
{
  if (!gps.fix_valid)
    return;
  if (std::isnan(gps.latitude_deg) || std::isnan(gps.longitude_deg))
    return;

  if (!origin_set_)
  {
    lat_origin_ = gps.latitude_deg;
    lon_origin_ = gps.longitude_deg;
    alt_origin_ = gps.altitude_m;
    origin_set_ = true;
    std::cout << "GPS Origin Set." << std::endl;
    return;
  }

  double deg_to_rad = M_PI / 180.0;
  double d_lat = gps.latitude_deg - lat_origin_;
  double d_lon = gps.longitude_deg - lon_origin_;
  double pos_x = d_lat * 111132.0;
  double pos_y = d_lon * 111132.0 * std::cos(lat_origin_ * deg_to_rad);
  double pos_z = gps.altitude_m - alt_origin_;

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
  R(0, 0) = 6.25;
  R(1, 1) = 6.25;
  R(2, 2) = 6.25;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  Eigen::VectorXd z(3);
  z << pos_x, pos_y, pos_z;
  Eigen::VectorXd y = z - x_.segment<3>(0);
  Eigen::MatrixXd S = (H * P_ * H.transpose()) + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  x_ = x_ + (K * y);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P_ = (I - (K * H)) * P_;
}

void Ekf::updateMag(const MagMeasurement &mag)
{
  if (std::isnan(mag.mag_x))
    return;

  double yaw_measured = std::atan2(mag.mag_y, mag.mag_x);

  double qw = x_(6);
  double qx = x_(7);
  double qy = x_(8);
  double qz = x_(9);
  double y_part = 2.0 * (qw * qz + qx * qy);
  double x_part = 1.0 - 2.0 * (qy * qy + qz * qz);
  double yaw_predicted = std::atan2(y_part, x_part);

  double innovation = yaw_measured - yaw_predicted;
  while (innovation > M_PI)
    innovation -= 2.0 * M_PI;
  while (innovation < -M_PI)
    innovation += 2.0 * M_PI;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
  double norm_sq = y_part * y_part + x_part * x_part;
  if (norm_sq < 1e-6)
    return;
  double d_atan = 1.0 / norm_sq;

  double dt1_dw = 2.0 * qz;
  double dt1_dx = 2.0 * qy;
  double dt1_dy = 2.0 * qx;
  double dt1_dz = 2.0 * qw;
  double dt2_dw = 0.0;
  double dt2_dx = 0.0;
  double dt2_dy = -4.0 * qy;
  double dt2_dz = -4.0 * qz;

  H(0, 6) = d_atan * (x_part * dt1_dw - y_part * dt2_dw);
  H(0, 7) = d_atan * (x_part * dt1_dx - y_part * dt2_dx);
  H(0, 8) = d_atan * (x_part * dt1_dy - y_part * dt2_dy);
  H(0, 9) = d_atan * (x_part * dt1_dz - y_part * dt2_dz);

  Eigen::MatrixXd R(1, 1);
  R << 0.1;
  Eigen::VectorXd z(1);
  z << innovation;
  Eigen::MatrixXd S = (H * P_ * H.transpose()) + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  x_ = x_ + (K * z);

  Eigen::Quaterniond q_updated(x_(6), x_(7), x_(8), x_(9));
  q_updated.normalize();
  x_(6) = q_updated.w();
  x_(7) = q_updated.x();
  x_(8) = q_updated.y();
  x_(9) = q_updated.z();

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P_ = (I - (K * H)) * P_;
}

Eigen::Vector3d Ekf::getPosition() const { return x_.segment<3>(0); }
Eigen::Vector3d Ekf::getVelocity() const { return x_.segment<3>(3); }
Eigen::Quaterniond Ekf::getOrientation() const { return Eigen::Quaterniond(x_(6), x_(7), x_(8), x_(9)); }