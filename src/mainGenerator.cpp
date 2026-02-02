#include "DataGenerator.hpp"
#include <iostream>

int main()
{
  std::cout << "Starting Simulation..." << std::endl;

  DataGenerator::Config config;
  config.duration_sec = 300.0;
  config.output_filename = "flight_data.bin";

  DataGenerator sim(config);
  sim.generate();

  std::cout << "Simulation Complete. File 'flight_data.bin' created." << std::endl;
  return 0;
}