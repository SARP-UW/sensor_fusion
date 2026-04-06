#include "Ekf.hpp"
#include <iostream>
#include <cmath>

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
  lat_origin_ = 0.0;
  lon_origin_ = 0.0;
  alt_origin_ = 0.0;

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

  if (dt <= 0.0)
    return;
  if (dt > 0.5)
    dt = 0.5;

  const Eigen::Vector3d pos = getPosition();
  const Eigen::Vector3d vel = getVelocity();
  const Eigen::Quaterniond q = getOrientation();

  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

  const Eigen::Vector3d omega(imu.gyro_x, imu.gyro_y, imu.gyro_z);
  Eigen::Quaterniond q_dot;
  q_dot.w() = 0.0;
  q_dot.vec() = omega;
  Eigen::Quaterniond q_new;
  q_new.coeffs() = q.coeffs() + (0.5 * dt) * (q * q_dot).coeffs();
  q_new.normalize();

  x_(6) = q_new.w();
  x_(7) = q_new.x();
  x_(8) = q_new.y();
  x_(9) = q_new.z();

  const Eigen::Vector3d acc_body(imu.acc_x, imu.acc_y, imu.acc_z);
  const double bx = acc_body.x(), by = acc_body.y(), bz = acc_body.z();

  const Eigen::Vector3d acc_world = q_new * acc_body;
  const Eigen::Vector3d gravity(0.0, 0.0, 9.81);
  const Eigen::Vector3d acc_net = acc_world - gravity;

  x_.segment<3>(3) = vel + acc_net * dt;
  x_.segment<3>(0) = pos + vel * dt + 0.5 * acc_net * (dt * dt);

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  Eigen::MatrixXd J_acc_q(3, 4);

  J_acc_q(0, 0) = -2 * qz * by + 2 * qy * bz;
  J_acc_q(0, 1) = 2 * qy * by + 2 * qz * bz;
  J_acc_q(0, 2) = -4 * qy * bx + 2 * qx * by + 2 * qw * bz;
  J_acc_q(0, 3) = -4 * qz * bx - 2 * qw * by + 2 * qx * bz;

  J_acc_q(1, 0) = 2 * qz * bx - 2 * qx * bz;
  J_acc_q(1, 1) = 2 * qy * bx - 4 * qx * by - 2 * qw * bz;
  J_acc_q(1, 2) = 2 * qx * bx + 2 * qz * bz;
  J_acc_q(1, 3) = 2 * qw * bx - 4 * qz * by + 2 * qy * bz;

  J_acc_q(2, 0) = -2 * qy * bx + 2 * qx * by;
  J_acc_q(2, 1) = 2 * qz * bx + 2 * qw * by - 4 * qx * bz;
  J_acc_q(2, 2) = -2 * qw * bx + 2 * qz * by - 4 * qy * bz;
  J_acc_q(2, 3) = 2 * qx * bx + 2 * qy * by;

  F.block<3, 4>(3, 6) = dt * J_acc_q;
  F.block<3, 4>(0, 6) = 0.5 * dt * dt * J_acc_q;

  const double omx = omega.x(), omy = omega.y(), omz = omega.z();
  Eigen::Matrix4d Omega;
  Omega << 0, -omx, -omy, -omz,
      omx, 0, omz, -omy,
      omy, -omz, 0, omx,
      omz, omy, -omx, 0;
  F.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() + 0.5 * dt * Omega;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.0001;
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.01;
  Q.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() * 0.001;

  P_ = F * P_ * F.transpose() + Q;

  const Eigen::Vector4d q_vec(q_new.w(), q_new.x(), q_new.y(), q_new.z());
  const Eigen::Matrix4d J_norm = Eigen::Matrix4d::Identity() - q_vec * q_vec.transpose();

  P_.block<4, 4>(6, 6) = J_norm * P_.block<4, 4>(6, 6) * J_norm.transpose();
  const Eigen::Matrix<double, 4, 6> cross = J_norm * P_.block<4, 6>(6, 0);
  P_.block<4, 6>(6, 0) = cross;
  P_.block<6, 4>(0, 6) = cross.transpose();
}

static void renormalizeQuat(Eigen::VectorXd &x)
{
  Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
  q.normalize();
  x(6) = q.w();
  x(7) = q.x();
  x(8) = q.y();
  x(9) = q.z();
}

void Ekf::updateBaro(const BaroMeasurement &baro)
{
  if (std::isnan(baro.pressure_Pa) || baro.pressure_Pa <= 0.0)
    return;

  const double P0 = 101325.0;
  const double measured_alt = 44330.0 * (1.0 - std::pow(baro.pressure_Pa / P0, 1.0 / 5.255));

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
  H(0, 2) = 1.0;

  Eigen::MatrixXd R(1, 1);
  R << 4.0;

  Eigen::VectorXd z(1);
  z << measured_alt;
  const Eigen::VectorXd y = z - H * x_;

  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();

  x_ = x_ + K * y;

  renormalizeQuat(x_);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  const Eigen::MatrixXd IKH = I - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
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

  const double deg_to_rad = M_PI / 180.0;
  const double pos_x = (gps.latitude_deg - lat_origin_) * 111132.0;
  const double pos_y = (gps.longitude_deg - lon_origin_) * 111132.0 * std::cos(lat_origin_ * deg_to_rad);
  const double pos_z = gps.altitude_m - alt_origin_;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 6.25;

  Eigen::VectorXd z(3);
  z << pos_x, pos_y, pos_z;
  const Eigen::VectorXd y = z - x_.segment<3>(0);

  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();

  x_ = x_ + K * y;

  renormalizeQuat(x_);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  const Eigen::MatrixXd IKH = I - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}

void Ekf::updateMag(const MagMeasurement &mag)
{
  if (std::isnan(mag.mag_x))
    return;

  const double yaw_measured = std::atan2(mag.mag_y, mag.mag_x);

  const double qw = x_(6), qx = x_(7), qy = x_(8), qz = x_(9);
  const double y_part = 2.0 * (qw * qz + qx * qy);
  const double x_part = 1.0 - 2.0 * (qy * qy + qz * qz);

  const double norm_sq = y_part * y_part + x_part * x_part;
  if (norm_sq < 1e-6)
    return;

  const double yaw_predicted = std::atan2(y_part, x_part);

  double innovation = yaw_measured - yaw_predicted;
  while (innovation > M_PI)
    innovation -= 2.0 * M_PI;
  while (innovation < -M_PI)
    innovation += 2.0 * M_PI;

  const double d_atan = 1.0 / norm_sq;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_DIM);
  H(0, 6) = d_atan * x_part * 2.0 * qz;
  H(0, 7) = d_atan * x_part * 2.0 * qy;
  H(0, 8) = d_atan * (x_part * 2.0 * qx + y_part * 4.0 * qy);
  H(0, 9) = d_atan * (x_part * 2.0 * qw + y_part * 4.0 * qz);

  Eigen::MatrixXd R(1, 1);
  R << 0.1;

  Eigen::VectorXd z(1);
  z << innovation;

  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();

  x_ = x_ + K * z;

  renormalizeQuat(x_);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  const Eigen::MatrixXd IKH = I - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}

Eigen::Vector3d Ekf::getPosition() const { return x_.segment<3>(0); }
Eigen::Vector3d Ekf::getVelocity() const { return x_.segment<3>(3); }
Eigen::Quaterniond Ekf::getOrientation() const { return Eigen::Quaterniond(x_(6), x_(7), x_(8), x_(9)); }