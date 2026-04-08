#include "Ekf.hpp"
#include <iostream>
#include <cmath>

static Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(),
      v.z(), 0, -v.x(),
      -v.y(), v.x(), 0;
  return S;
}

static Eigen::Matrix<double, 4, 3> Xi(const Eigen::Quaterniond &q)
{
  Eigen::Matrix<double, 4, 3> xi;
  xi << -q.x(), -q.y(), -q.z(),
      q.w(), -q.z(), q.y(),
      q.z(), q.w(), -q.x(),
      -q.y(), q.x(), q.w();
  return xi;
}

Ekf::Ekf()
{
  x_ = Eigen::VectorXd::Zero(NOM_DIM);
  x_(6) = 1.0;

  P_ = Eigen::MatrixXd::Zero(ERR_DIM, ERR_DIM);
  P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1;
  P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.1;
  P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.1;
  P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.01;
  P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.1;

  last_timestamp_sec_ = 0.0;
  initialized_ = false;
  origin_set_ = false;
  lat_origin_ = 0.0;
  lon_origin_ = 0.0;
  alt_origin_ = 0.0;

  std::cout << "EKF Initialized (MEKF, 15-state error covariance)." << std::endl;
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

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ERR_DIM, ERR_DIM);
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q_pos;
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_vel;
  Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * q_att;
  Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * q_bg;
  Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * q_ba;
  return Q;
}

void Ekf::applyCorrection(const Eigen::VectorXd &dx)
{
  x_.segment<3>(0) += dx.segment<3>(0);
  x_.segment<3>(3) += dx.segment<3>(3);
  x_.segment<3>(10) += dx.segment<3>(9);
  x_.segment<3>(13) += dx.segment<3>(12);

  const Eigen::Vector3d d_theta = dx.segment<3>(6);
  const Eigen::Quaterniond q_nom(x_(6), x_(7), x_(8), x_(9));
  const Eigen::Quaterniond dq(1.0, d_theta.x() / 2.0,
                              d_theta.y() / 2.0,
                              d_theta.z() / 2.0);
  const Eigen::Quaterniond q_new = (q_nom * dq).normalized();
  x_(6) = q_new.w();
  x_(7) = q_new.x();
  x_(8) = q_new.y();
  x_(9) = q_new.z();
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

  const Eigen::Matrix3d R_q = q_new.toRotationMatrix();

  const Eigen::Vector3d acc_world = R_q * acc_b;
  const Eigen::Vector3d gravity(0.0, 0.0, 9.81);
  const Eigen::Vector3d acc_net = acc_world - gravity;
  x_.segment<3>(3) = vel + acc_net * dt;
  x_.segment<3>(0) = pos + vel * dt + 0.5 * acc_net * (dt * dt);

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(ERR_DIM, ERR_DIM);

  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

  const Eigen::Matrix3d J_att = -R_q * skew(acc_b);
  F.block<3, 3>(3, 6) = dt * J_att;
  F.block<3, 3>(0, 6) = 0.5 * dt * dt * J_att;

  F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(omega) * dt;

  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

  F.block<3, 3>(3, 12) = -dt * R_q;
  F.block<3, 3>(0, 12) = -0.5 * dt * dt * R_q;

  const Eigen::MatrixXd Q = getProcessNoise(phase_estimator_.getPhase());
  P_ = F * P_ * F.transpose() + Q;

  phase_estimator_.update(getPosition().z(), getVelocity().z(),
                          acc_raw.norm(), dt);
}

void Ekf::updateWithMeasurement(const Eigen::VectorXd &innovation,
                                const Eigen::MatrixXd &H,
                                const Eigen::MatrixXd &R)
{
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();
  const Eigen::VectorXd dx = K * innovation;

  applyCorrection(dx);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ERR_DIM, ERR_DIM);
  const Eigen::MatrixXd IKH = I - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}

void Ekf::updateBaro(const BaroMeasurement &baro)
{
  if (std::isnan(baro.pressure_Pa) || baro.pressure_Pa <= 0.0)
    return;

  const double P0 = 101325.0;
  const double measured_alt = 44330.0 * (1.0 - std::pow(baro.pressure_Pa / P0, 1.0 / 5.255));

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, ERR_DIM);
  H(0, 2) = 1.0;

  Eigen::MatrixXd R(1, 1);
  R << 4.0;

  const Eigen::VectorXd y = (Eigen::VectorXd(1) << measured_alt - x_(2)).finished();
  updateWithMeasurement(y, H, R);
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

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, ERR_DIM);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 6.25;

  Eigen::VectorXd z(3);
  z << pos_x, pos_y, pos_z;
  const Eigen::VectorXd y = z - x_.segment<3>(0);
  updateWithMeasurement(y, H, R);
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

  double innovation = yaw_measured - std::atan2(y_part, x_part);
  while (innovation > M_PI)
    innovation -= 2.0 * M_PI;
  while (innovation < -M_PI)
    innovation += 2.0 * M_PI;

  const double d = 1.0 / norm_sq;
  Eigen::Matrix<double, 1, 4> H_q;
  H_q(0, 0) = d * x_part * 2.0 * qz;
  H_q(0, 1) = d * x_part * 2.0 * qy;
  H_q(0, 2) = d * (x_part * 2.0 * qx + y_part * 4.0 * qy);
  H_q(0, 3) = d * (x_part * 2.0 * qw + y_part * 4.0 * qz);

  const Eigen::Quaterniond q_nom(qw, qx, qy, qz);
  const Eigen::Matrix<double, 1, 3> H_att = H_q * (0.5 * Xi(q_nom));

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, ERR_DIM);
  H.block<1, 3>(0, 6) = H_att;

  Eigen::MatrixXd R(1, 1);
  R << 0.1;
  const Eigen::VectorXd y = (Eigen::VectorXd(1) << innovation).finished();
  updateWithMeasurement(y, H, R);
}

Eigen::Vector3d Ekf::getPosition() const { return x_.segment<3>(0); }
Eigen::Vector3d Ekf::getVelocity() const { return x_.segment<3>(3); }
Eigen::Quaterniond Ekf::getOrientation() const { return Eigen::Quaterniond(x_(6), x_(7), x_(8), x_(9)); }
Eigen::Vector3d Ekf::getGyroBias() const { return x_.segment<3>(10); }
Eigen::Vector3d Ekf::getAccelBias() const { return x_.segment<3>(13); }
FlightPhase Ekf::getPhase() const { return phase_estimator_.getPhase(); }