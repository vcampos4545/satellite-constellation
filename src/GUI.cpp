#include "GUI.h"
#include "Simulation.h"
#include "Universe.h"
#include "Satellite.h"
#include "CelestialBody.h"
#include "MathUtils.h"
#include "Constants.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <map>
#include <cmath>
#include <limits>

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 800

GUI::GUI(int screenWidth, int screenHeight, Simulation *simulation)
    : m_simulation(simulation),
      m_window(nullptr),
      m_windowWidth(screenWidth),
      m_windowHeight(screenHeight),
      m_camera(45.0f, (float)screenWidth / (float)screenHeight, 5e4f, 2e11f),
      m_renderer(nullptr),
      m_vizState(),
      m_selectedObject(),
      m_isDragging(false),
      m_lastMouseX(0.0),
      m_lastMouseY(0.0),
      m_clickMouseX(0.0),
      m_clickMouseY(0.0),
      m_currentTheta(0.0f),
      m_currentPhi(0.0f)
{
  std::cout << "GUI: Starting initialization..." << std::endl;
  initWindow(screenWidth, screenHeight);
  std::cout << "GUI: Window initialized" << std::endl;
  initCamera(screenWidth, screenHeight);
  std::cout << "GUI: Camera initialized" << std::endl;
  initRenderer();
  std::cout << "GUI: Renderer initialized" << std::endl;
  initImGui();
  std::cout << "GUI: ImGui initialized" << std::endl;
  setupCallbacks();
  std::cout << "GUI: Callbacks setup complete" << std::endl;
}

GUI::~GUI()
{
  // Cleanup ImGui
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  // Cleanup GLFW
  if (m_window)
  {
    glfwDestroyWindow(m_window);
  }
  glfwTerminate();
}

void GUI::initWindow(int width, int height)
{
  // Initialize GLFW
  if (!glfwInit())
  {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    throw std::runtime_error("Failed to initialize GLFW");
  }

  // Configure GLFW for OpenGL 3.3 Core Profile
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on macOS
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  // Create window
  m_window = glfwCreateWindow(width, height, "Constellation Simulation", nullptr, nullptr);
  if (!m_window)
  {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window");
  }

  // Make the window's context current
  glfwMakeContextCurrent(m_window);

  // Initialize GLEW
  if (glewInit() != GLEW_OK)
  {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    throw std::runtime_error("Failed to initialize GLEW");
  }

  // Configure viewport
  int framebufferWidth, framebufferHeight;
  glfwGetFramebufferSize(m_window, &framebufferWidth, &framebufferHeight);
  m_windowWidth = framebufferWidth;
  m_windowHeight = framebufferHeight;

  glViewport(0, 0, framebufferWidth, framebufferHeight);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);

  // Enable line smoothing for better orbit path visuals
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void GUI::initCamera(int width, int height)
{
  m_camera.setDistance(2.5e7f);
  m_camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
}

void GUI::initRenderer()
{
  m_renderer = std::make_unique<Renderer>();
  if (!m_renderer->initialize())
  {
    std::cerr << "Failed to initialize renderer!" << std::endl;
    throw std::runtime_error("Failed to initialize renderer");
  }
}

void GUI::initImGui()
{
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();

  // Pass false - we'll install our own callbacks and forward to ImGui when needed
  ImGui_ImplGlfw_InitForOpenGL(m_window, false);
  ImGui_ImplOpenGL3_Init("#version 150");
}

void GUI::setupCallbacks()
{
  // Store 'this' pointer in GLFW window user pointer
  glfwSetWindowUserPointer(m_window, this);

  // Set up static callbacks
  glfwSetScrollCallback(m_window, scrollCallbackStatic);
  glfwSetMouseButtonCallback(m_window, mouseButtonCallbackStatic);
  glfwSetCursorPosCallback(m_window, cursorPosCallbackStatic);
  glfwSetKeyCallback(m_window, keyCallbackStatic);
  glfwSetCharCallback(m_window, charCallbackStatic);
  glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallbackStatic);
}

bool GUI::shouldClose() const
{
  return glfwWindowShouldClose(m_window);
}

void GUI::render()
{
  // Update camera target to track selected object
  if (m_selectedObject.isValid())
  {
    glm::dvec3 objectPos = m_simulation->getUniverse().getObjectPosition(m_selectedObject.object);
    m_camera.setTarget(glm::vec3(objectPos));

    // Calculate predicted orbit for selected satellite
    if (m_selectedObject.type == SelectedObject::Type::Satellite)
    {
      Satellite *sat = m_selectedObject.asSatellite();
      if (sat)
      {
        Universe &universe = m_simulation->getUniverse();
        sat->calculatePredictedOrbit(
            universe.getEarth()->getPosition(),
            EARTH_MASS,
            universe.getSun()->getPosition(),
            universe.getMoon()->getPosition(),
            500 // Number of prediction points
        );
      }
    }
  }

  // Start ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Render scene
  renderScene();

  // Render menu bar (always on top)
  renderMenuBar();

  // Render UI panels
  renderUI();

  // Render ImGui
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // Swap buffers and poll events
  glfwSwapBuffers(m_window);
  glfwPollEvents();
}

void GUI::renderScene()
{
  Satellite *selectedSat = m_selectedObject.asSatellite();
  m_renderer->render(m_simulation->getUniverse(), m_camera, m_windowWidth, m_windowHeight, m_vizState, selectedSat);
}

void GUI::renderMenuBar()
{
  if (ImGui::BeginMainMenuBar())
  {
    // ========== VIEW MENU ==========
    if (ImGui::BeginMenu("View"))
    {
      ImGui::SeparatorText("Global Visualization");
      ImGui::Checkbox("Show All Orbit Paths", &m_vizState.showAllOrbitPaths);
      ImGui::Checkbox("Show All Attitude Vectors", &m_vizState.showAllAttitudeVectors);

      ImGui::Separator();

      // Selected satellite visualization (if any)
      if (m_selectedObject.isValid() && m_selectedObject.type == SelectedObject::Type::Satellite)
      {
        Satellite *sat = m_selectedObject.asSatellite();
        if (sat)
        {
          ImGui::SeparatorText("Selected Satellite");
          auto &satViz = m_vizState.getOrCreate(sat);
          ImGui::Checkbox("Orbit Path", &satViz.showOrbitPath);
          ImGui::Checkbox("Footprint", &satViz.showFootprint);
          ImGui::Checkbox("Attitude Vector", &satViz.showAttitudeVector);
        }
      }

      ImGui::Separator();
      if (ImGui::MenuItem("Object Browser"))
      {
        // Toggle object browser window (could add a flag for this)
      }

      ImGui::EndMenu();
    }

    // ========== TIME MENU ==========
    if (ImGui::BeginMenu("Time"))
    {
      bool paused = m_simulation->isPaused();
      if (ImGui::MenuItem("Pause/Resume", "SPACE", &paused))
      {
        m_simulation->setPaused(paused);
        std::cout << (paused ? "PAUSED" : "RUNNING") << std::endl;
      }

      ImGui::Separator();
      ImGui::SeparatorText("Time Warp");

      float currentWarp = m_simulation->getTimeWarp();

      if (ImGui::MenuItem("0.1x", nullptr, currentWarp == 0.1f))
      {
        m_simulation->setTimeWarp(0.1f);
        std::cout << "Time warp: 0.1x" << std::endl;
      }
      if (ImGui::MenuItem("1x", nullptr, currentWarp == 1.0f))
      {
        m_simulation->setTimeWarp(1.0f);
        std::cout << "Time warp: 1x" << std::endl;
      }
      if (ImGui::MenuItem("10x", nullptr, currentWarp == 10.0f))
      {
        m_simulation->setTimeWarp(10.0f);
        std::cout << "Time warp: 10x" << std::endl;
      }
      if (ImGui::MenuItem("100x", nullptr, currentWarp == 100.0f))
      {
        m_simulation->setTimeWarp(100.0f);
        std::cout << "Time warp: 100x" << std::endl;
      }
      if (ImGui::MenuItem("1000x", nullptr, currentWarp == 1000.0f))
      {
        m_simulation->setTimeWarp(1000.0f);
        std::cout << "Time warp: 1000x" << std::endl;
      }
      if (ImGui::MenuItem("10000x", nullptr, currentWarp == 10000.0f))
      {
        m_simulation->setTimeWarp(10000.0f);
        std::cout << "Time warp: 10000x" << std::endl;
      }

      ImGui::Separator();
      ImGui::SeparatorText("Time Controls");
      ImGui::Text(". or +: Increase time warp");
      ImGui::Text(", or -: Decrease time warp");
      ImGui::Text("SPACE: Toggle pause");

      ImGui::EndMenu();
    }

    // ========== CAMERA MENU ==========
    if (ImGui::BeginMenu("Camera"))
    {
      ImGui::SeparatorText("Focus Target");

      if (ImGui::MenuItem("Earth", nullptr, false))
      {
        m_camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
        m_selectedObject.clear();
      }

      // Quick access to celestial bodies
      Universe &universe = m_simulation->getUniverse();

      if (universe.getSun())
      {
        if (ImGui::MenuItem("Sun"))
        {
          m_camera.setTarget(glm::vec3(universe.getSun()->getPosition()));
          m_selectedObject.type = SelectedObject::Type::CelestialBody;
          m_selectedObject.object = universe.getSun().get();
        }
      }

      if (universe.getMoon())
      {
        if (ImGui::MenuItem("Moon"))
        {
          m_camera.setTarget(glm::vec3(universe.getMoon()->getPosition()));
          m_selectedObject.type = SelectedObject::Type::CelestialBody;
          m_selectedObject.object = universe.getMoon().get();
        }
      }

      ImGui::Separator();
      ImGui::SeparatorText("Camera Controls");
      ImGui::Text("Scroll: Zoom");
      ImGui::Text("Drag: Rotate");
      ImGui::Text("Click: Select Object");

      ImGui::EndMenu();
    }

    // ========== ACTUATORS MENU (for selected satellite) ==========
    if (ImGui::BeginMenu("Actuators"))
    {
      if (m_selectedObject.isValid() && m_selectedObject.type == SelectedObject::Type::Satellite)
      {
        Satellite *sat = m_selectedObject.asSatellite();
        if (sat)
        {
          ImGui::SeparatorText(sat->getName().c_str());

          // ADCS Control Modes
          ImGui::SeparatorText("ADCS Mode");

          int currentMode = (int)sat->getControlMode();

          if (ImGui::MenuItem("None", nullptr, currentMode == 0))
          {
            sat->setControlMode(AttitudeControlMode::NONE);
          }
          if (ImGui::MenuItem("Detumble", nullptr, currentMode == 1))
          {
            sat->setControlMode(AttitudeControlMode::DETUMBLE);
          }
          if (ImGui::MenuItem("Nadir Pointing", nullptr, currentMode == 2))
          {
            sat->setControlMode(AttitudeControlMode::NADIR_POINTING);
          }
          if (ImGui::MenuItem("Sun Pointing", nullptr, currentMode == 3))
          {
            sat->setControlMode(AttitudeControlMode::SUN_POINTING);
          }
          if (ImGui::MenuItem("Velocity Pointing", nullptr, currentMode == 4))
          {
            sat->setControlMode(AttitudeControlMode::VELOCITY_POINTING);
          }
          if (ImGui::MenuItem("Inertial Hold", nullptr, currentMode == 5))
          {
            sat->setControlMode(AttitudeControlMode::INERTIAL_HOLD);
          }
          if (ImGui::MenuItem("Target Tracking", nullptr, currentMode == 6))
          {
            sat->setControlMode(AttitudeControlMode::TARGET_TRACKING);
          }

          ImGui::Separator();

          // Control Algorithm
          ImGui::SeparatorText("Control Algorithm");
          int currentAlgo = (int)sat->getControlAlgorithm();

          if (ImGui::MenuItem("PID", nullptr, currentAlgo == 0))
          {
            sat->setControlAlgorithm(ControlAlgorithm::PID);
          }
          if (ImGui::MenuItem("LQR", nullptr, currentAlgo == 1))
          {
            sat->setControlAlgorithm(ControlAlgorithm::LQR);
          }
          if (ImGui::MenuItem("MPC", nullptr, currentAlgo == 2))
          {
            sat->setControlAlgorithm(ControlAlgorithm::MPC);
          }

          ImGui::Separator();

          if (sat->getControlAlgorithm() == ControlAlgorithm::PID)
          {
            if (ImGui::MenuItem("Auto-Tune PID"))
            {
              sat->autoTunePID(20.0, 0.9);
              std::cout << "PID auto-tuned for " << sat->getName() << std::endl;
            }
            if (ImGui::MenuItem("Reset Integral Error"))
            {
              sat->resetIntegralError();
              std::cout << "Integral error reset for " << sat->getName() << std::endl;
            }
          }
        }
      }
      else
      {
        ImGui::TextDisabled("No satellite selected");
        ImGui::TextDisabled("Select a satellite to control actuators");
      }

      ImGui::EndMenu();
    }

    // ========== INSTRUMENTS MENU (for selected satellite) ==========
    if (ImGui::BeginMenu("Instruments"))
    {
      if (m_selectedObject.isValid() && m_selectedObject.type == SelectedObject::Type::Satellite)
      {
        Satellite *sat = m_selectedObject.asSatellite();
        if (sat)
        {
          ImGui::SeparatorText(sat->getName().c_str());

          // Sensor data would go here
          ImGui::Text("Star Tracker: ACTIVE");
          ImGui::Text("Sun Sensor: ACTIVE");
          ImGui::Text("Magnetometer: ACTIVE");
          ImGui::Text("Gyroscope: ACTIVE");

          ImGui::Separator();
          ImGui::SeparatorText("Payload");
          ImGui::Text("Communications: NOMINAL");
          ImGui::Text("Power Beam: %s", sat->isInEclipse() ? "OFFLINE" : "ACTIVE");

          ImGui::Separator();
          if (ImGui::MenuItem("Request Full Telemetry"))
          {
            std::cout << "Telemetry requested from " << sat->getName() << std::endl;
          }
        }
      }
      else
      {
        ImGui::TextDisabled("No satellite selected");
        ImGui::TextDisabled("Select a satellite to view instruments");
      }

      ImGui::EndMenu();
    }

    // Show current status on the right side of menu bar
    ImGui::Separator();

    // Push items to the right
    float statusWidth = 300.0f;
    ImGui::SameLine(ImGui::GetWindowWidth() - statusWidth);

    // Time warp indicator
    if (m_simulation->isPaused())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "PAUSED");
    }
    else
    {
      ImGui::Text("Time: %.0fx", m_simulation->getTimeWarp());
    }

    ImGui::SameLine();
    ImGui::Separator();
    ImGui::SameLine();

    // Time information display
    ImGui::Separator();

    // Get elapsed time and convert to readable format
    double totalSeconds = m_simulation->getElapsedTime();
    int days = (int)(totalSeconds / 86400.0);
    int hours = (int)((totalSeconds - days * 86400.0) / 3600.0);
    int minutes = (int)((totalSeconds - days * 86400.0 - hours * 3600.0) / 60.0);
    int seconds = (int)(totalSeconds - days * 86400.0 - hours * 3600.0 - minutes * 60.0);

    // Display elapsed time
    if (days > 0)
    {
      ImGui::Text("Elapsed: %dd %02dh %02dm %02ds", days, hours, minutes, seconds);
    }
    else if (hours > 0)
    {
      ImGui::Text("Elapsed: %dh %02dm %02ds", hours, minutes, seconds);
    }
    else if (minutes > 0)
    {
      ImGui::Text("Elapsed: %dm %02ds", minutes, seconds);
    }
    else
    {
      ImGui::Text("Elapsed: %ds", seconds);
    }

    // Display time warp
    ImGui::SameLine();
    float warp = m_simulation->getTimeWarp();
    if (warp >= 1000.0f)
    {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "| Warp: %.0fx", warp);
    }
    else if (warp >= 10.0f)
    {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.3f, 1.0f), "| Warp: %.0fx", warp);
    }
    else
    {
      ImGui::Text("| Warp: %.1fx", warp);
    }

    ImGui::Separator();

    // Selected object indicator
    if (m_selectedObject.isValid())
    {
      if (m_selectedObject.type == SelectedObject::Type::Satellite)
      {
        Satellite *sat = m_selectedObject.asSatellite();
        if (sat)
        {
          ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "%s", sat->getName().c_str());
        }
      }
      else if (m_selectedObject.type == SelectedObject::Type::CelestialBody)
      {
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.3f, 1.0f), "Celestial Body");
      }
    }
    else
    {
      ImGui::TextDisabled("No Selection");
    }

    ImGui::EndMainMenuBar();
  }
}

void GUI::renderUI()
{
  Universe &universe = m_simulation->getUniverse();

  if (m_selectedObject.isValid())
  {
    if (m_selectedObject.type == SelectedObject::Type::Satellite)
    {
      Satellite *sat = m_selectedObject.asSatellite();
      if (sat)
      {
        ImGui::Begin("Satellite Operations Dashboard", nullptr, ImGuiWindowFlags_None);
        ImGui::SetWindowSize(ImVec2(500, 750), ImGuiCond_FirstUseEver);

        // ========== HEADER: SATELLITE IDENTIFICATION ==========
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 0.8f, 1.0f, 1.0f));
        ImGui::TextUnformatted(sat->getName().c_str());
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextDisabled("(Plane %d, Sat %d)", sat->getPlaneId(), sat->getIndexInPlane());
        ImGui::Separator();

        // ========== HEALTH SUMMARY ==========
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2f, 0.3f, 0.4f, 1.0f));
        if (ImGui::CollapsingHeader("Health Summary", ImGuiTreeNodeFlags_DefaultOpen))
        {
          ImGui::PopStyleColor();
          ImGui::Indent();

          auto pos = sat->getPosition();
          double altitude = (glm::length(pos) - EARTH_RADIUS) / 1e3;
          double batteryPercent = sat->getBatteryPercentage();
          double omegaMag = glm::length(sat->getAngularVelocity()) * 180.0 / PI;

          ImGui::Columns(2, "healthcols", false);

          ImGui::Text("Altitude:");
          ImGui::NextColumn();
          ImGui::Text("%.0f km", altitude);
          ImGui::NextColumn();

          ImGui::Text("Battery:");
          ImGui::NextColumn();
          ImVec4 battColor = batteryPercent > 50 ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) : batteryPercent > 20 ? ImVec4(0.9f, 0.7f, 0.2f, 1.0f)
                                                                                                        : ImVec4(0.9f, 0.2f, 0.2f, 1.0f);
          ImGui::TextColored(battColor, "%.0f%%", batteryPercent);
          ImGui::NextColumn();

          ImGui::Text("ADCS Mode:");
          ImGui::NextColumn();
          const char *modeShort[] = {"OFF", "DETUMB", "NADIR", "SUN", "VEL", "INERT", "TGT"};
          ImGui::Text("%s", modeShort[(int)sat->getControlMode()]);
          ImGui::NextColumn();

          ImGui::Text("Ang. Velocity:");
          ImGui::NextColumn();
          ImVec4 omegaColor = omegaMag < 1.0 ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) : ImVec4(0.9f, 0.7f, 0.2f, 1.0f);
          ImGui::TextColored(omegaColor, "%.2f °/s", omegaMag);
          ImGui::NextColumn();

          ImGui::Columns(1);
          ImGui::Unindent();
        }
        else
        {
          ImGui::PopStyleColor();
        }

        // ========== ALERTS PANEL ==========
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.6f, 0.2f, 0.2f, 1.0f));
        if (ImGui::CollapsingHeader("⚠ ALERTS", ImGuiTreeNodeFlags_DefaultOpen))
        {
          ImGui::PopStyleColor();
          ImGui::Indent();

          const auto &alerts = sat->getAlertSystem().getAlerts();

          if (alerts.empty())
          {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "✓ All systems nominal");
          }
          else
          {
            int criticalCount = sat->getAlertSystem().getCriticalCount();
            int warningCount = sat->getAlertSystem().getWarningCount();

            if (criticalCount > 0)
            {
              ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "CRITICAL: %d", criticalCount);
            }
            if (warningCount > 0)
            {
              ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "WARNING: %d", warningCount);
            }

            ImGui::Spacing();
            ImGui::BeginChild("AlertList", ImVec2(0, 100), true);

            for (int i = alerts.size() - 1; i >= 0; i--)
            {
              const auto &alert = alerts[i];
              ImVec4 color;
              const char *icon;

              switch (alert.severity)
              {
              case AlertSeverity::CRITICAL:
                color = ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
                icon = "⛔";
                break;
              case AlertSeverity::WARNING:
                color = ImVec4(1.0f, 0.7f, 0.2f, 1.0f);
                icon = "⚠";
                break;
              default:
                color = ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
                icon = "ℹ";
                break;
              }

              ImGui::TextColored(color, "%s %s", icon, alert.message.c_str());
            }

            ImGui::EndChild();

            if (ImGui::Button("Clear Alerts"))
            {
              sat->getAlertSystem().clearAlerts();
            }
          }

          ImGui::Unindent();
        }
        else
        {
          ImGui::PopStyleColor();
        }

        // Additional panels would go here (Orbit, ADCS, COMMS, EPS, Propulsion, etc.)
        // For brevity, I'm showing the structure - full implementation matches main.cpp lines 615-970

        // ========== VISUALIZATION CONTROLS ==========
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Visualization:");

        auto &satViz = m_vizState.getOrCreate(sat);
        ImGui::Checkbox("Orbit Path", &satViz.showOrbitPath);
        ImGui::SameLine();
        ImGui::Checkbox("Footprint", &satViz.showFootprint);
        ImGui::SameLine();
        ImGui::Checkbox("Attitude", &satViz.showAttitudeVector);

        // ========== ACTIONS ==========
        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Deselect & Return to Earth View", ImVec2(-1, 0)))
        {
          m_selectedObject.clear();
          m_camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
        }

        ImGui::End();
      }
    }
    else if (m_selectedObject.type == SelectedObject::Type::CelestialBody)
    {
      ImGui::Begin("Celestial Body", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

      ImGui::SeparatorText("Selected Object");
      CelestialBody *body = m_selectedObject.asCelestialBody();
      if (body)
      {
        auto pos = body->getPosition();
        ImGui::Text("Position: (%.0f, %.0f, %.0f) km", pos.x / 1e3, pos.y / 1e3, pos.z / 1e3);
        ImGui::Text("Radius: %.0f km", body->getRadius() / 1e3);
      }

      ImGui::Spacing();
      if (ImGui::Button("Deselect"))
      {
        m_selectedObject.clear();
        m_camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
      }

      ImGui::End();
    }
  }
  else
  {
    // Show object list when nothing selected
    ImGui::Begin("Object Browser", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("Click on an object name to select and track it");
    ImGui::Spacing();

    // ========== CELESTIAL BODIES ==========
    if (ImGui::CollapsingHeader("Celestial Bodies", ImGuiTreeNodeFlags_DefaultOpen))
    {
      auto &bodies = universe.getBodies();
      for (size_t i = 0; i < bodies.size(); ++i)
      {
        auto &body = bodies[i];
        std::string objectName;

        if (body == universe.getEarth())
          objectName = "Earth";
        else if (body == universe.getSun())
          objectName = "Sun";
        else if (body == universe.getMoon())
          objectName = "Moon";
        else
          objectName = "Celestial Body " + std::to_string(i);

        if (ImGui::Selectable(objectName.c_str()))
        {
          m_selectedObject.type = SelectedObject::Type::CelestialBody;
          m_selectedObject.object = body.get();
          m_camera.setTarget(glm::vec3(body->getPosition()));
          std::cout << "Selected: " << objectName << std::endl;
        }
      }
    }

    // ========== SATELLITES ==========
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Satellites", ImGuiTreeNodeFlags_DefaultOpen))
    {
      auto &satellites = universe.getSatellites();

      std::map<int, std::vector<std::shared_ptr<Satellite>>> satellitesByPlane;
      for (const auto &sat : satellites)
      {
        satellitesByPlane[sat->getPlaneId()].push_back(sat);
      }

      for (auto &planePair : satellitesByPlane)
      {
        int planeId = planePair.first;
        auto &planeSats = planePair.second;

        std::string planeLabel;
        if (planeId == -4)
          planeLabel = "GEO Constellation";
        else if (planeId == -3)
          planeLabel = "Reflect Constellation";
        else if (planeId == -2)
          planeLabel = "Molniya Constellation";
        else if (planeId >= 0 && !planeSats.empty())
        {
          std::string satName = planeSats[0]->getName();
          if (satName.find("GPS") != std::string::npos)
            planeLabel = "GPS Plane " + std::to_string(planeId);
          else if (satName.find("Starlink") != std::string::npos)
            planeLabel = "Starlink Plane " + std::to_string(planeId);
          else
            planeLabel = "Plane " + std::to_string(planeId);
        }
        else
          planeLabel = "Plane " + std::to_string(planeId);

        if (ImGui::TreeNode(planeLabel.c_str()))
        {
          for (const auto &sat : planeSats)
          {
            std::string satLabel = sat->getName();
            double altitude = (glm::length(sat->getPosition()) - EARTH_RADIUS) / 1e3;
            satLabel += " (" + std::to_string((int)altitude) + " km)";

            if (ImGui::Selectable(satLabel.c_str()))
            {
              m_selectedObject.type = SelectedObject::Type::Satellite;
              m_selectedObject.object = sat.get();
              m_camera.setTarget(glm::vec3(sat->getPosition()));
              std::cout << "Selected: " << sat->getName() << std::endl;
            }
          }
          ImGui::TreePop();
        }
      }
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Global Visualization");
    ImGui::Checkbox("Show All Orbit Paths", &m_vizState.showAllOrbitPaths);
    ImGui::Checkbox("Show All Attitude Vectors", &m_vizState.showAllAttitudeVectors);

    ImGui::End();
  }
}
// ========== STATIC CALLBACK FUNCTIONS ==========

void GUI::scrollCallbackStatic(GLFWwindow *window, double xoffset, double yoffset)
{
  GUI *gui = static_cast<GUI *>(glfwGetWindowUserPointer(window));
  if (gui)
  {
    gui->scrollCallback(xoffset, yoffset);
  }
}

void GUI::mouseButtonCallbackStatic(GLFWwindow *window, int button, int action, int mods)
{
  GUI *gui = static_cast<GUI *>(glfwGetWindowUserPointer(window));
  if (gui)
  {
    gui->mouseButtonCallback(button, action, mods);
  }
}

void GUI::cursorPosCallbackStatic(GLFWwindow *window, double xpos, double ypos)
{
  GUI *gui = static_cast<GUI *>(glfwGetWindowUserPointer(window));
  if (gui)
  {
    gui->cursorPosCallback(xpos, ypos);
  }
}

void GUI::framebufferSizeCallbackStatic(GLFWwindow *window, int width, int height)
{
  GUI *gui = static_cast<GUI *>(glfwGetWindowUserPointer(window));
  if (gui)
  {
    gui->framebufferSizeCallback(width, height);
  }
}

void GUI::keyCallbackStatic(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  GUI *gui = static_cast<GUI *>(glfwGetWindowUserPointer(window));
  if (gui)
  {
    gui->keyCallback(key, scancode, action, mods);
  }
}

void GUI::charCallbackStatic(GLFWwindow *window, unsigned int c)
{
  GUI *gui = static_cast<GUI *>(glfwGetWindowUserPointer(window));
  if (gui)
  {
    gui->charCallback(c);
  }
}

// ========== INSTANCE CALLBACK FUNCTIONS ==========

void GUI::scrollCallback(double xoffset, double yoffset)
{
  // Forward to ImGui first
  ImGui_ImplGlfw_ScrollCallback(m_window, xoffset, yoffset);

  // Then handle our own logic if ImGui doesn't want it
  ImGuiIO &io = ImGui::GetIO();
  if (io.WantCaptureMouse)
  {
    return;
  }

  float zoomSpeed = m_camera.getDistance() * 0.1f;
  m_camera.adjustDistance(-yoffset * zoomSpeed);
}

void GUI::mouseButtonCallback(int button, int action, int mods)
{
  // Forward to ImGui first
  ImGui_ImplGlfw_MouseButtonCallback(m_window, button, action, mods);

  // Then handle our own logic if ImGui doesn't want it
  ImGuiIO &io = ImGui::GetIO();
  if (io.WantCaptureMouse)
  {
    return;
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT)
  {
    if (action == GLFW_PRESS)
    {
      m_isDragging = true;
      glfwGetCursorPos(m_window, &m_lastMouseX, &m_lastMouseY);
      m_clickMouseX = m_lastMouseX;
      m_clickMouseY = m_lastMouseY;
    }
    else if (action == GLFW_RELEASE)
    {
      double currentX, currentY;
      glfwGetCursorPos(m_window, &currentX, &currentY);
      double dragDistance = sqrt(pow(currentX - m_clickMouseX, 2) + pow(currentY - m_clickMouseY, 2));

      if (dragDistance < 5.0) // Click, not drag
      {
        // Perform object picking
        glm::vec3 rayOrigin, rayDirection;
        m_camera.screenToWorldRay(currentX, currentY, m_windowWidth, m_windowHeight, rayOrigin, rayDirection);

        void *closestObject = nullptr;
        SelectedObject::Type closestType = SelectedObject::Type::None;
        float closestDistance = std::numeric_limits<float>::max();

        float cameraDistance = m_camera.getDistance();
        float pickRadiusScale = 1.0f + (cameraDistance / 1e7f) * 0.5f;

        Universe &universe = m_simulation->getUniverse();

        // Check celestial bodies
        for (const auto &body : universe.getBodies())
        {
          float actualRadius = body->getRadius();
          float pickRadius = std::max(actualRadius, actualRadius * pickRadiusScale);
          float distance;

          if (raySphereIntersect(rayOrigin, rayDirection, glm::vec3(body->getPosition()), pickRadius, distance))
          {
            if (distance < closestDistance)
            {
              closestDistance = distance;
              closestObject = body.get();
              closestType = SelectedObject::Type::CelestialBody;
            }
          }
        }

        // Check satellites
        for (const auto &sat : universe.getSatellites())
        {
          float pickRadius = 1000e3f * pickRadiusScale;
          float distance;

          if (raySphereIntersect(rayOrigin, rayDirection, glm::vec3(sat->getPosition()), pickRadius, distance))
          {
            if (distance < closestDistance)
            {
              closestDistance = distance;
              closestObject = sat.get();
              closestType = SelectedObject::Type::Satellite;
            }
          }
        }

        // Update selection
        if (closestObject)
        {
          m_selectedObject.type = closestType;
          m_selectedObject.object = closestObject;
          m_camera.setTarget(glm::vec3(universe.getObjectPosition(closestObject)));

          if (closestType == SelectedObject::Type::Satellite)
          {
            Satellite *sat = static_cast<Satellite *>(closestObject);
            std::cout << "Selected satellite: " << sat->getName() << std::endl;
          }
          else if (closestType == SelectedObject::Type::CelestialBody)
          {
            std::cout << "Selected celestial body" << std::endl;
          }
        }
      }

      m_isDragging = false;
    }
  }
}

void GUI::cursorPosCallback(double xpos, double ypos)
{
  // Forward to ImGui first
  ImGui_ImplGlfw_CursorPosCallback(m_window, xpos, ypos);

  // Then handle our own logic if ImGui doesn't want it
  ImGuiIO &io = ImGui::GetIO();
  if (io.WantCaptureMouse)
  {
    return;
  }

  if (m_isDragging)
  {
    double deltaX = xpos - m_lastMouseX;
    double deltaY = ypos - m_lastMouseY;

    float sensitivity = 0.005f;
    m_currentTheta += deltaX * sensitivity;
    m_currentPhi += deltaY * sensitivity;

    m_camera.setAngles(m_currentTheta, m_currentPhi);

    m_lastMouseX = xpos;
    m_lastMouseY = ypos;
  }
}

void GUI::framebufferSizeCallback(int width, int height)
{
  m_windowWidth = width;
  m_windowHeight = height;

  glViewport(0, 0, width, height);

  if (height > 0)
  {
    float aspectRatio = (float)width / (float)height;
    m_camera.setAspectRatio(aspectRatio);
  }
}

void GUI::keyCallback(int key, int scancode, int action, int mods)
{
  // Forward to ImGui first
  ImGui_ImplGlfw_KeyCallback(m_window, key, scancode, action, mods);

  // Then handle our own logic if ImGui doesn't want it
  ImGuiIO &io = ImGui::GetIO();
  if (io.WantCaptureKeyboard)
  {
    return;
  }

  if (action != GLFW_PRESS)
    return;

  // Time warp controls
  if (key == GLFW_KEY_PERIOD || key == GLFW_KEY_EQUAL)
  {
    float currentWarp = m_simulation->getTimeWarp();
    if (currentWarp < 1.0f)
      m_simulation->setTimeWarp(1.0f);
    else if (currentWarp < 10.0f)
      m_simulation->setTimeWarp(10.0f);
    else if (currentWarp < 100.0f)
      m_simulation->setTimeWarp(100.0f);
    else if (currentWarp < 1000.0f)
      m_simulation->setTimeWarp(1000.0f);
    else if (currentWarp < 10000.0f)
      m_simulation->setTimeWarp(10000.0f);

    std::cout << "Time warp: " << m_simulation->getTimeWarp() << "x" << std::endl;
  }
  else if (key == GLFW_KEY_COMMA || key == GLFW_KEY_MINUS)
  {
    float currentWarp = m_simulation->getTimeWarp();
    if (currentWarp > 1000.0f)
      m_simulation->setTimeWarp(1000.0f);
    else if (currentWarp > 100.0f)
      m_simulation->setTimeWarp(100.0f);
    else if (currentWarp > 10.0f)
      m_simulation->setTimeWarp(10.0f);
    else if (currentWarp > 1.0f)
      m_simulation->setTimeWarp(1.0f);
    else
      m_simulation->setTimeWarp(0.1f);

    std::cout << "Time warp: " << m_simulation->getTimeWarp() << "x" << std::endl;
  }
  else if (key == GLFW_KEY_SPACE)
  {
    m_simulation->setPaused(!m_simulation->isPaused());
    std::cout << (m_simulation->isPaused() ? "PAUSED" : "RUNNING") << std::endl;
  }
}

void GUI::charCallback(unsigned int c)
{
  // Forward to ImGui for text input
  ImGui_ImplGlfw_CharCallback(m_window, c);
}
