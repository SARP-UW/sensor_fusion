#include "Ekf.hpp"
#include <iostream>
#include <cmath>

Ekf::Ekf()
{
  x_ = Eigen::VectorXd::Zero(STATE_DIM);
  x_(6) = 1.0;

  P_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1;
  P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.1;
  P_.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() * 0.1;
  P_.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * 0.01;
  P_.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * 0.1;

  last_timestamp_sec_ = 0.0;
  initialized_ = false;
  origin_set_ = false;
  lat_origin_ = 0.0;
  lon_origin_ = 0.0;
  alt_origin_ = 0.0;

  std::cout << "EKF Initialized (16-state with bias tracking)." << std::endl;
}

Eigen::MatrixXd Ekf::getProcessNoise(FlightPhase phase) const
{
  double q_pos, q_vel, q_att, q_bg, q_ba;
  switch (phase)
  {
  case FlightPhase::PRE_LAUNCH:
    q_pos = 1e-6;
    q_vel = 1e-4;
    q_att = 1e-5;
    q_bg = 1e-8;
    q_ba = 1e-7;
    break;
  case FlightPhase::BOOST:
    q_pos = 1e-4;
    q_vel = 0.1;
    q_att = 1e-3;
    q_bg = 1e-7;
    q_ba = 1e-5;
    break;
  case FlightPhase::COAST:
    q_pos = 1e-5;
    q_vel = 0.01;
    q_att = 5e-4;
    q_bg = 1e-8;
    q_ba = 1e-7;
    break;
  case FlightPhase::DESCENT:
    q_pos = 1e-5;
    q_vel = 1e-3;
    q_att = 5e-4;
    q_bg = 1e-8;
    q_ba = 1e-7;
    break;
  case FlightPhase::LANDED:
    q_pos = 1e-8;
    q_vel = 1e-6;
    q_att = 1e-6;
    q_bg = 1e-9;
    q_ba = 1e-9;
    break;
  default:
    q_pos = 1e-4;
    q_vel = 0.01;
    q_att = 1e-3;
    q_bg = 1e-7;
    q_ba = 1e-6;
    break;
  }
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q_pos;
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_vel;
  Q.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() * q_att;
  Q.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * q_bg;
  Q.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * q_ba;
  return Q;
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
  const Eigen::Vector3d b_g = getGyroBias();
  const Eigen::Vector3d b_a = getAccelBias();
  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

  const Eigen::Vector3d omega_raw(imu.gyro_x, imu.gyro_y, imu.gyro_z);
  const Eigen::Vector3d acc_raw(imu.acc_x, imu.acc_y, imu.acc_z);
  const Eigen::Vector3d omega = omega_raw - b_g;
  const Eigen::Vector3d acc_b = acc_raw - b_a;

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

  const Eigen::Vector3d acc_world = q_new * acc_b;
  const Eigen::Vector3d gravity(0.0, 0.0, 9.81);
  const Eigen::Vector3d acc_net = acc_world - gravity;
  x_.segment<3>(3) = vel + acc_net * dt;
  x_.segment<3>(0) = pos + vel * dt + 0.5 * acc_net * (dt * dt);

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  const double bx = acc_b.x(), by = acc_b.y(), bz = acc_b.z();
  Eigen::Matrix<double, 3, 4> J_acc_q;
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
  Eigen::Matrix4d OmegaMat;
  OmegaMat << 0, -omx, -omy, -omz,
      omx, 0, omz, -omy,
      omy, -omz, 0, omx,
      omz, omy, -omx, 0;
  F.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() + 0.5 * dt * OmegaMat;

  Eigen::Matrix<double, 4, 3> Xi_q;
  Xi_q << -qx, -qy, -qz,
      qw, -qz, qy,
      qz, qw, -qx,
      -qy, qx, qw;
  F.block<4, 3>(6, 10) = -0.5 * dt * Xi_q;

  const Eigen::Matrix3d R_q = q_new.toRotationMatrix();
  F.block<3, 3>(3, 13) = -dt * R_q;
  F.block<3, 3>(0, 13) = -0.5 * dt * dt * R_q;

  const Eigen::MatrixXd Q = getProcessNoise(phase_estimator_.getPhase());
  P_ = F * P_ * F.transpose() + Q;

  const Eigen::Vector4d q_vec(q_new.w(), q_new.x(), q_new.y(), q_new.z());
  const Eigen::Matrix4d J_norm = Eigen::Matrix4d::Identity() - q_vec * q_vec.transpose();

  P_.block<4, 4>(6, 6) = J_norm * P_.block<4, 4>(6, 6) * J_norm.transpose();
  const Eigen::Matrix<double, 4, 6> cross_pv = J_norm * P_.block<4, 6>(6, 0);
  P_.block<4, 6>(6, 0) = cross_pv;
  P_.block<6, 4>(0, 6) = cross_pv.transpose();
  const Eigen::Matrix<double, 4, 6> cross_b = J_norm * P_.block<4, 6>(6, 10);
  P_.block<4, 6>(6, 10) = cross_b;
  P_.block<6, 4>(10, 6) = cross_b.transpose();

  phase_estimator_.update(getPosition().z(), getVelocity().z(),
                          acc_raw.norm(), dt);
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

  const Eigen::VectorXd y = (Eigen::VectorXd(1) << measured_alt).finished() - H * x_;
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();

  x_ = x_ + K * y;
  renormalizeQuat(x_);
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
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
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
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

  double innovation = std::atan2(y_part, x_part);
  innovation = yaw_measured - innovation;
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
  const Eigen::VectorXd z = (Eigen::VectorXd(1) << innovation).finished();
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();

  x_ = x_ + K * z;
  renormalizeQuat(x_);
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}

Eigen::Vector3d Ekf::getPosition() const { return x_.segment<3>(0); }
Eigen::Vector3d Ekf::getVelocity() const { return x_.segment<3>(3); }
Eigen::Quaterniond Ekf::getOrientation() const { return Eigen::Quaterniond(x_(6), x_(7), x_(8), x_(9)); }
Eigen::Vector3d Ekf::getGyroBias() const { return x_.segment<3>(10); }
Eigen::Vector3d Ekf::getAccelBias() const { return x_.segment<3>(13); }
FlightPhase Ekf::getPhase() const { return phase_estimator_.getPhase(); }