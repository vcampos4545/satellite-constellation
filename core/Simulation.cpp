#include "Simulation.h"
#include "GUI.h"
#include <GLFW/glfw3.h>
#include <cstdio>

Simulation::Simulation(bool headless)
    : m_headless(headless),
      m_isPaused(false),
      m_timeWarpMultiplier(1.0f),
      m_elapsedTime(0.0),
      m_universe(),
      m_gui(nullptr)
{
  printf("\033[32mSimulation: Universe initialized\033[0m\n");
  if (!m_headless)
  {
    printf("\033[32mSimulation: Creating GUI...\033[0m\n");
    m_gui = std::make_unique<GUI>(1000, 800, this);
    printf("\033[32mSimulation: GUI created successfully\033[0m\n");
  }
}

Simulation::~Simulation()
{
  // GUI unique_ptr will automatically clean up
}

void Simulation::run()
{
  printf("\n\033[32m=== SIMULATION STARTED ===\033[0m\n");

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

  printf("\n\033[32m=== SIMULATION ENDED ===\033[0m\n");
}

void Simulation::update(float deltaTime)
{
  if (!m_isPaused)
  {
    float warpedDeltaTime = deltaTime * m_timeWarpMultiplier;
    // Update orbital physics with sub-stepping (max 0.1 second physics steps)
    m_universe.update(warpedDeltaTime, 0.1);

    // Track elapsed simulation time (warped time)
    m_elapsedTime += warpedDeltaTime;
  }
}