/*
Satellite Constellation Simulation

This simulation displays the Earth and a constellation of satellites.
Simulates space-based solar power satellites beaming energy to Earth.

*/

#include "Simulation.h"
#include <iostream>

int main()
{
  try
  {
    // Create simulation (with GUI by default)
    Simulation sim(false); // false = not headless, show GUI

    // Run simulation
    sim.run();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
}
