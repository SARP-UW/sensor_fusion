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

  bool operator<(const SensorEvent &other) const
  {
    return timestamp < other.timestamp;
  }
};

int main()
{
  std::string input_file = "flight_data.bin";
  std::cout << "Loading " << input_file << "..." << std::endl;

  DataIngestion loader;
  ParsedData data = loader.loadFromFile(input_file);

  if (data.imu_data.empty())
  {
    std::cerr << "Error: No data found. Did you run ./generate_data?" << std::endl;
    return 1;
  }

  std::vector<SensorEvent> timeline;

  for (const auto &imu : data.imu_data)
  {
    SensorEvent e;
    e.timestamp = imu.timestamp_sec;
    e.type = 0;
    e.imu = &imu;
    timeline.push_back(e);
  }

  for (const auto &baro : data.baro_data)
  {
    SensorEvent e;
    e.timestamp = baro.timestamp_sec;
    e.type = 1;
    e.baro = &baro;
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

  std::ofstream out_file("trajectory.csv");
  out_file << "time,pos_x,pos_y,pos_z,vel_z,quat_w,quat_x,quat_y,quat_z\n";

  Ekf filter;

  for (const auto &event : timeline)
  {
    if (event.type == 0)
    {
      filter.predict(*event.imu);
    }
    else if (event.type == 1)
    {
      filter.updateBaro(*event.baro);
    }
    else if (event.type == 2)
    {
      filter.updateGps(*event.gps);
    }
    else if (event.type == 3)
    {
      filter.updateMag(*event.mag);
    }

    Eigen::Vector3d pos = filter.getPosition();
    Eigen::Vector3d vel = filter.getVelocity();
    Eigen::Quaterniond q = filter.getOrientation();

    out_file << std::fixed << std::setprecision(4)
             << event.timestamp << ","
             << pos.x() << "," << pos.y() << "," << pos.z() << ","
             << vel.z() << ","
             << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "\n";
  }

  out_file.close();
  std::cout << "Done! Results written to 'trajectory.csv'." << std::endl;

  return 0;
}