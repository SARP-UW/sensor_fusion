#include "gtest/gtest.h"
#include "Ekf.hpp"

TEST(EkfTest, ShouldInitializeWithZeroState)
{
  Ekf filter;

  // Check Position (Should be 0,0,0).
  Eigen::Vector3d pos = filter.getPosition();
  EXPECT_NEAR(pos.x(), 0.0, 1e-9);
  EXPECT_NEAR(pos.y(), 0.0, 1e-9);
  EXPECT_NEAR(pos.z(), 0.0, 1e-9);

  // Check Velocity (Should be 0,0,0).
  Eigen::Vector3d vel = filter.getVelocity();
  EXPECT_NEAR(vel.x(), 0.0, 1e-9);

  // Check Orientation (Should be Identity: w=1, x=0, y=0, z=0).
  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_NEAR(q.w(), 1.0, 1e-9);
  EXPECT_NEAR(q.x(), 0.0, 1e-9);
}

TEST(EkfTest, ShouldIntegrateVerticalAcceleration)
{
  Ekf filter;

  // Simulate 1 second of flight
  double t = 0.0;
  double dt = 0.01; // 100 Hz

  // We want the rocket to accelerate UP at 10 m/s^2 (kinematic).
  // Gravity is 9.81.
  // So the IMU should read (10 + 9.81) = 19.81 on the Z axis.
  double sim_acc_z = 19.81;

  for (int i = 0; i < 101; ++i)
  {
    ImuMeasurement imu;
    imu.timestamp_sec = t;
    imu.sensor_id = 0;
    imu.acc_x = 0;
    imu.acc_y = 0;
    imu.acc_z = sim_acc_z;
    imu.gyro_x = 0;
    imu.gyro_y = 0;
    imu.gyro_z = 0;

    filter.predict(imu);
    t += dt;
  }

  // 1. Check Velocity
  // v = a * t = 10 * 1.0 = 10.0 m/s
  Eigen::Vector3d vel = filter.getVelocity();
  EXPECT_NEAR(vel.z(), 10.0, 0.1);

  // 2. Check Position
  // p = 0.5 * a * t^2 = 0.5 * 10 * 1.0 = 5.0 meters
  Eigen::Vector3d pos = filter.getPosition();
  EXPECT_NEAR(pos.z(), 5.0, 0.1);
}

TEST(EkfTest, ShouldCorrectAltitudeWithBarometer)
{
  Ekf filter;

  // 1. Initial State: Altitude = 0m
  Eigen::Vector3d initial_pos = filter.getPosition();
  EXPECT_NEAR(initial_pos.z(), 0.0, 0.1);

  // 2. Barometer Reading: 100m
  // We need to back-calculate the Pressure for 100m altitude
  // P = P0 * (1 - h/44330)^5.255
  double target_alt = 100.0;
  double P0 = 101325.0;
  double target_pressure = P0 * std::pow(1.0 - (target_alt / 44330.0), 5.255);

  BaroMeasurement baro;
  baro.timestamp_sec = 0.1;
  baro.sensor_id = 0;
  baro.pressure_Pa = target_pressure;

  // 3. Update
  filter.updateBaro(baro);

  // 4. Verify Correction
  // The filter should have moved the altitude UP towards 100m.
  // It won't be exactly 100m (because it still trusts its initial 0m estimate a little bit),
  // but it should be significantly greater than 0.
  Eigen::Vector3d new_pos = filter.getPosition();

  // Expecting the filter to move towards the measurement
  EXPECT_GT(new_pos.z(), 1.0);
  EXPECT_LT(new_pos.z(), 105.0);
}

TEST(EkfTest, ShouldUpdateWithMagnetometer)
{
  Ekf filter;

  // 1. Initial State: Yaw is 0 (Quaternion 1,0,0,0)
  // 2. Sensor Reading: "North" (X=20, Y=0) -> Yaw should be 0.
  MagMeasurement mag;
  mag.timestamp_sec = 0.1;
  mag.sensor_id = 0;
  mag.mag_x = 20.0;
  mag.mag_y = 0.0;
  mag.mag_z = 40.0;

  // 3. Run Update
  filter.updateMag(mag);

  // 4. Verify State is Finite (Not NaN)
  Eigen::Quaterniond q = filter.getOrientation();
  EXPECT_TRUE(std::isfinite(q.w()));
  EXPECT_TRUE(std::isfinite(q.x()));
  EXPECT_TRUE(std::isfinite(q.y()));
  EXPECT_TRUE(std::isfinite(q.z()));

  // 5. Test a case with an error (Rocket is 0 deg, Mag says 90 deg)
  mag.mag_x = 0.0;
  mag.mag_y = 20.0;

  filter.updateMag(mag);

  // Filter should rotate towards 90 deg (positive Z component in quaternion)
  q = filter.getOrientation();
  EXPECT_GT(q.z(), 0.001);
  EXPECT_TRUE(std::isfinite(q.w()));
}