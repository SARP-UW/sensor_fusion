#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include "DataIngestion.hpp"
#include "Ekf.hpp"

struct SensorEvent
{
  double timestamp;
  int type;
  const ImuMeasurement *imu = nullptr;
  const BaroMeasurement *baro = nullptr;
  const GpsMeasurement *gps = nullptr;
  const MagMeasurement *mag = nullptr;

  bool operator<(const SensorEvent &o) const { return timestamp < o.timestamp; }
};

int main()
{
  const std::string input_file = "flight_data.bin";
  std::cout << "Loading " << input_file << "..." << std::endl;

  DataIngestion loader;
  ParsedData data = loader.loadFromFile(input_file);

  if (data.imu_data.empty())
  {
    std::cerr << "Error: No IMU data found. Did you run ./src/generate_data?" << std::endl;
    return 1;
  }

  std::vector<SensorEvent> timeline;
  timeline.reserve(data.imu_data.size() + data.baro_data.size() +
                   data.gps_data.size() + data.mag_data.size());

  for (const auto &d : data.imu_data)
  {
    SensorEvent e;
    e.timestamp = d.timestamp_sec;
    e.type = 0;
    e.imu = &d;
    timeline.push_back(e);
  }
  for (const auto &d : data.baro_data)
  {
    SensorEvent e;
    e.timestamp = d.timestamp_sec;
    e.type = 1;
    e.baro = &d;
    timeline.push_back(e);
  }
  for (const auto &d : data.gps_data)
  {
    SensorEvent e;
    e.timestamp = d.timestamp_sec;
    e.type = 2;
    e.gps = &d;
    timeline.push_back(e);
  }
  for (const auto &d : data.mag_data)
  {
    SensorEvent e;
    e.timestamp = d.timestamp_sec;
    e.type = 3;
    e.mag = &d;
    timeline.push_back(e);
  }

  std::sort(timeline.begin(), timeline.end());
  std::cout << "Processing " << timeline.size() << " events..." << std::endl;

  std::ofstream out("trajectory.csv");
  out << "time,"
      << "pos_x,pos_y,pos_z,"
      << "vel_x,vel_y,vel_z,"
      << "quat_w,quat_x,quat_y,quat_z,"
      << "bg_x,bg_y,bg_z,"
      << "ba_x,ba_y,ba_z,"
      << "phase\n";

  Ekf filter;

  for (const auto &event : timeline)
  {
    if (event.type == 0)
      filter.predict(*event.imu);
    else if (event.type == 1)
      filter.updateBaro(*event.baro);
    else if (event.type == 2)
      filter.updateGps(*event.gps);
    else if (event.type == 3)
      filter.updateMag(*event.mag);

    const Eigen::Vector3d pos = filter.getPosition();
    const Eigen::Vector3d vel = filter.getVelocity();
    const Eigen::Quaterniond q = filter.getOrientation();
    const Eigen::Vector3d bg = filter.getGyroBias();
    const Eigen::Vector3d ba = filter.getAccelBias();

    out << std::fixed << std::setprecision(6)
        << event.timestamp << ","
        << pos.x() << "," << pos.y() << "," << pos.z() << ","
        << vel.x() << "," << vel.y() << "," << vel.z() << ","
        << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
        << bg.x() << "," << bg.y() << "," << bg.z() << ","
        << ba.x() << "," << ba.y() << "," << ba.z() << ","
        << flightPhaseToString(filter.getPhase()) << "\n";
  }

  out.close();
  std::cout << "Done. Results written to 'trajectory.csv'." << std::endl;
  return 0;
}