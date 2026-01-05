#include "Simulation.h"
#include "GUI.h"
#include <GLFW/glfw3.h>
#include <iostream>

Simulation::Simulation(bool headless)
    : m_headless(headless),
      m_isPaused(false),
      m_timeWarpMultiplier(1.0f),
      m_universe(),
      m_gui(nullptr)
{
  std::cout << "Simulation: Universe initialized" << std::endl;
  if (!m_headless)
  {
    std::cout << "Simulation: Creating GUI..." << std::endl;
    m_gui = std::make_unique<GUI>(1000, 800, this);
    std::cout << "Simulation: GUI created successfully" << std::endl;
  }
}

Simulation::~Simulation()
{
  // GUI unique_ptr will automatically clean up
}

void Simulation::run()
{
  std::cout << "\n=== SIMULATION STARTED ===" << std::endl;
  if (m_headless)
  {
    std::cout << "Running in headless mode (no GUI)" << std::endl;
  }
  else
  {
    std::cout << "Running with GUI" << std::endl;
    std::cout << "\n=== CONTROLS ===" << std::endl;
    std::cout << "Mouse Click: Select satellite and track with camera" << std::endl;
    std::cout << "Mouse Drag: Pan camera" << std::endl;
    std::cout << "Mouse Scroll: Zoom in/out" << std::endl;
    std::cout << "SPACE: Pause/Unpause simulation" << std::endl;
    std::cout << ". or +: Increase time warp (1x -> 10x -> 100x -> 1000x -> 10000x)" << std::endl;
    std::cout << ", or -: Decrease time warp" << std::endl;
    std::cout << "================" << std::endl;
  }

  std::cout << "Simulation::run() - About to start main loop" << std::endl;

  float lastTime = m_headless ? 0.0f : glfwGetTime();

  // Main simulation loop
  while (true)
  {
    // Check if GUI window should close
    if (!m_headless && m_gui->shouldClose())
    {
      break;
    }

    // Calculate delta time
    float currentTime = m_headless ? 0.0f : glfwGetTime();
    float deltaTime = m_headless ? 0.016f : (currentTime - lastTime); // 60 FPS for headless
    lastTime = currentTime;

    // Update simulation
    update(deltaTime);

    // Render GUI (if not headless)
    if (!m_headless)
    {
      m_gui->render();
    }

    // For headless mode, add exit condition (e.g., max time, user input, etc.)
    // For now, headless will run indefinitely until Ctrl+C
  }

  std::cout << "\n=== SIMULATION ENDED ===" << std::endl;
}

void Simulation::update(float deltaTime)
{
  if (!m_isPaused)
  {
    float warpedDeltaTime = deltaTime * m_timeWarpMultiplier;
    // Update orbital physics with sub-stepping (max 0.1 second physics steps)
    m_universe.update(warpedDeltaTime, 0.1);
  }
}