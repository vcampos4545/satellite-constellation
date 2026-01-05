/*
Main simulation class.

This class starts the simulation with run method.
This can run headless without a GUI.
*/

#ifndef SIMULATION_H
#define SIMULATION_H

#include "Universe.h"
#include <memory>

// Forward declaration to avoid circular dependency
class GUI;

class Simulation
{
public:
  Simulation(bool headless = false);
  ~Simulation();

  void run();

  // Simulation control (called by GUI)
  void setPaused(bool paused) { m_isPaused = paused; }
  bool isPaused() const { return m_isPaused; }
  void setTimeWarp(float multiplier) { m_timeWarpMultiplier = multiplier; }
  float getTimeWarp() const { return m_timeWarpMultiplier; }

  // Access to universe (for GUI to render/query)
  Universe& getUniverse() { return m_universe; }
  const Universe& getUniverse() const { return m_universe; }

private:
  bool m_headless;
  bool m_isPaused;
  float m_timeWarpMultiplier;

  Universe m_universe;
  std::unique_ptr<GUI> m_gui;

  void update(float deltaTime);
};

#endif // SIMULATION_H