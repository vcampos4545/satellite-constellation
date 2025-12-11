/*
Satellite Constelation

This simulation aims to display acurately the earth and a constelation of satelites.

These are simulated space based solar power satellites, beaming energy to earth at various locations

*/

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <map>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Constants.h"
#include "Universe.h"
#include "Camera.h"
#include "Renderer.h"
#include "MathUtils.h"
#include "Satellite.h"
#include "CelestialBody.h"
#include "VisualizationState.h"

// ImGui includes
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 800

// Input state structure
struct State
{
  Camera *camera;
  Universe *universe;
  bool isDragging;
  double lastMouseX;
  double lastMouseY;
  double clickMouseX; // Mouse position when clicked
  double clickMouseY; // Mouse position when clicked
  float currentTheta;
  float currentPhi;
  float *timeWarpMultiplier;      // Pointer to time warp multiplier
  bool *isPaused;                 // Pointer to pause state
  int windowWidth;                // Current window width
  int windowHeight;               // Current window height
  SelectedObject *selectedObject; // Currently selected object
  VisualizationState *vizState;   // Visualization state
};

// Mouse scroll callback for zooming
void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
  ImGuiIO &io = ImGui::GetIO();
  if (io.WantCaptureMouse)
  {
    return;
  }

  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (state && state->camera)
  {
    // Zoom speed depends on current distance
    float zoomSpeed = state->camera->getDistance() * 0.1f;
    state->camera->adjustDistance(-yoffset * zoomSpeed);
  }
}

// Mouse button callback for starting/stopping pan and satellite picking
void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (state)
  {
    // Skip if ImGui wants to capture mouse
    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureMouse)
    {
      return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
      if (action == GLFW_PRESS)
      {
        state->isDragging = true;
        glfwGetCursorPos(window, &state->lastMouseX, &state->lastMouseY);
        state->clickMouseX = state->lastMouseX;
        state->clickMouseY = state->lastMouseY;
      }
      else if (action == GLFW_RELEASE)
      {
        // Check if this was a click (minimal mouse movement) vs a drag
        double currentX, currentY;
        glfwGetCursorPos(window, &currentX, &currentY);
        double dragDistance = sqrt(pow(currentX - state->clickMouseX, 2) + pow(currentY - state->clickMouseY, 2));

        if (dragDistance < 5.0) // Less than 5 pixels moved = click
        {
          // Perform object picking
          glm::vec3 rayOrigin, rayDirection;
          state->camera->screenToWorldRay(currentX, currentY, state->windowWidth, state->windowHeight,
                                          rayOrigin, rayDirection);

          // Find closest intersected object
          void *closestObject = nullptr;
          SelectedObject::Type closestType = SelectedObject::Type::None;
          float closestDistance = std::numeric_limits<float>::max();

          // Calculate pick radius scaling based on camera distance
          // Makes objects easier to click when zoomed out
          float cameraDistance = state->camera->getDistance();
          float pickRadiusScale = 1.0f + (cameraDistance / 1e7f) * 0.5f; // Scale up to 1.5x when far away

          // Check celestial bodies (Earth, Sun, Moon)
          for (const auto &body : state->universe->getBodies())
          {
            // Use actual radius for large bodies, but ensure minimum pick size
            float actualRadius = body->getRadius();
            float pickRadius = std::max(actualRadius, actualRadius * pickRadiusScale);
            float distance;

            if (raySphereIntersect(rayOrigin, rayDirection,
                                   glm::vec3(body->getPosition()), pickRadius, distance))
            {
              if (distance < closestDistance)
              {
                closestDistance = distance;
                closestObject = body.get();
                closestType = SelectedObject::Type::CelestialBody;
              }
            }
          }

          // Check satellites (with much larger pick radius for easier clicking)
          for (const auto &sat : state->universe->getSatellites())
          {
            // Base pick radius: 1000 km, scaled by camera distance
            float pickRadius = 1000e3f * pickRadiusScale; // Much larger than before!
            float distance;

            if (raySphereIntersect(rayOrigin, rayDirection,
                                   glm::vec3(sat->getPosition()), pickRadius, distance))
            {
              if (distance < closestDistance)
              {
                closestDistance = distance;
                closestObject = sat.get();
                closestType = SelectedObject::Type::Satellite;
              }
            }
          }

          // Update selected object and camera target
          if (closestObject)
          {
            state->selectedObject->type = closestType;
            state->selectedObject->object = closestObject;
            state->camera->setTarget(glm::vec3(state->universe->getObjectPosition(closestObject)));

            // Print selection info
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

        state->isDragging = false;
      }
    }
  }
}

// Cursor position callback for panning
void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
  ImGuiIO &io = ImGui::GetIO();
  if (io.WantCaptureMouse)
  {
    return;
  }

  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (state && state->isDragging && state->camera)
  {
    // Calculate mouse delta
    double deltaX = xpos - state->lastMouseX;
    double deltaY = ypos - state->lastMouseY;

    // Update angles (sensitivity factor)
    float sensitivity = 0.005f;
    state->currentTheta += deltaX * sensitivity;
    state->currentPhi += deltaY * sensitivity;

    // Update camera
    state->camera->setAngles(state->currentTheta, state->currentPhi);

    // Store current position
    state->lastMouseX = xpos;
    state->lastMouseY = ypos;
  }
}

// Framebuffer size callback for handling window resizing
void framebufferSizeCallback(GLFWwindow *window, int width, int height)
{
  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (state)
  {
    // Update stored window dimensions
    state->windowWidth = width;
    state->windowHeight = height;

    // Update viewport
    glViewport(0, 0, width, height);

    // Update camera aspect ratio
    if (state->camera && height > 0)
    {
      float aspectRatio = (float)width / (float)height;
      state->camera->setAspectRatio(aspectRatio);
    }
  }
}

// Keyboard callback for time warp controls
void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  if (action != GLFW_PRESS)
    return;

  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (!state)
    return;

  // Time warp controls
  if (key == GLFW_KEY_PERIOD || key == GLFW_KEY_EQUAL) // '.' or '+' to speed up
  {
    if (*state->timeWarpMultiplier < 1.0f)
      *state->timeWarpMultiplier = 1.0f;
    else if (*state->timeWarpMultiplier < 10.0f)
      *state->timeWarpMultiplier = 10.0f;
    else if (*state->timeWarpMultiplier < 100.0f)
      *state->timeWarpMultiplier = 100.0f;
    else if (*state->timeWarpMultiplier < 1000.0f)
      *state->timeWarpMultiplier = 1000.0f;
    else if (*state->timeWarpMultiplier < 10000.0f)
      *state->timeWarpMultiplier = 10000.0f;

    std::cout << "Time warp: " << *state->timeWarpMultiplier << "x" << std::endl;
  }
  else if (key == GLFW_KEY_COMMA || key == GLFW_KEY_MINUS) // ',' or '-' to slow down
  {
    if (*state->timeWarpMultiplier > 1000.0f)
      *state->timeWarpMultiplier = 1000.0f;
    else if (*state->timeWarpMultiplier > 100.0f)
      *state->timeWarpMultiplier = 100.0f;
    else if (*state->timeWarpMultiplier > 10.0f)
      *state->timeWarpMultiplier = 10.0f;
    else if (*state->timeWarpMultiplier > 1.0f)
      *state->timeWarpMultiplier = 1.0f;
    else
      *state->timeWarpMultiplier = 0.1f;

    std::cout << "Time warp: " << *state->timeWarpMultiplier << "x" << std::endl;
  }
  else if (key == GLFW_KEY_SPACE) // Space to pause/unpause
  {
    *state->isPaused = !(*state->isPaused);
    std::cout << (*state->isPaused ? "PAUSED" : "RUNNING") << std::endl;
  }
}

GLFWwindow *initGUI(int screenWidth = 800, int screenHeight = 600)
{
  // Initialize GLFW
  if (!glfwInit())
  {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return nullptr; // Changed from -1
  }

  // Configure GLFW for OpenGL 3.3 Core Profile
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on macOS
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);           // Make window resizable

  // Create a windowed mode window and its OpenGL context
  GLFWwindow *window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Constelation", NULL, NULL);
  if (!window)
  {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return nullptr; // Changed from -1
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLEW
  if (glewInit() != GLEW_OK)
  {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return nullptr; // Changed from -1
  }

  // Configure viewport using actual framebuffer size (handles high-DPI displays)
  int framebufferWidth, framebufferHeight;
  glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
  glViewport(0, 0, framebufferWidth, framebufferHeight);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);

  // Enable line smoothing for better orbit path visuals
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  return window;
}

int main()
{
  // Init gui
  GLFWwindow *window = initGUI(SCREEN_WIDTH, SCREEN_HEIGHT);

  // Init simulation (Customize simulation in universe init)
  Universe universe;

  // Create camera
  Camera camera(45.0f, (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 5e4f, 2e11f); // Far plane extended to 200 million km to see the sun
  camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
  camera.setDistance(1.5e7f); // 15,000 km from Earth - good view of LEO constellation

  // Time control state
  float timeWarpMultiplier = 1.0f; // Start at 1x speed
  bool isPaused = false;

  // Selected object and visualization state
  SelectedObject selectedObject;
  VisualizationState vizState;

  // Initialize input state
  State state;
  state.camera = &camera;
  state.universe = &universe;
  state.isDragging = false;
  state.lastMouseX = 0.0;
  state.lastMouseY = 0.0;
  state.clickMouseX = 0.0;
  state.clickMouseY = 0.0;
  state.currentTheta = 0.0f;
  state.currentPhi = glm::pi<float>() / 4.0f; // Match initial camera phi
  state.timeWarpMultiplier = &timeWarpMultiplier;
  state.isPaused = &isPaused;
  state.selectedObject = &selectedObject;
  state.vizState = &vizState;

  // Initialize window dimensions in state
  int framebufferWidth, framebufferHeight;
  glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
  state.windowWidth = framebufferWidth;
  state.windowHeight = framebufferHeight;

  // Set up input callbacks
  glfwSetWindowUserPointer(window, &state);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetCursorPosCallback(window, cursorPosCallback);
  glfwSetKeyCallback(window, keyCallback);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

  // Initialize renderer (loads shaders, textures, creates meshes)
  Renderer renderer;
  if (!renderer.initialize())
  {
    std::cerr << "Failed to initialize renderer!" << std::endl;
    glfwTerminate();
    return -1;
  }

  // Initialize ImGui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();

  // Initialize ImGui for GLFW + OpenGL3
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // Print controls
  std::cout << "\n=== CONTROLS ===" << std::endl;
  std::cout << "Mouse Click: Select satellite and track with camera" << std::endl;
  std::cout << "Mouse Drag: Pan camera" << std::endl;
  std::cout << "Mouse Scroll: Zoom in/out" << std::endl;
  std::cout << "SPACE: Pause/Unpause simulation" << std::endl;
  std::cout << ". or +: Increase time warp (1x -> 10x -> 100x -> 1000x -> 10000x)" << std::endl;
  std::cout << ", or -: Decrease time warp" << std::endl;
  std::cout << "================\n"
            << std::endl;

  // Time tracking for rotation
  float lastTime = glfwGetTime();
  // Track time for updating window title
  float lastTitleUpdateTime = lastTime;
  float titleUpdateInterval = 0.5f; // Update title every 0.5 seconds

  // Main Simulation loop
  while (!glfwWindowShouldClose(window))
  {
    /*-------------------- UPDATE --------------------*/
    // Update simulation state
    float currentTime = glfwGetTime();
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // Apply time warp and pause
    if (!isPaused)
    {
      float warpedDeltaTime = deltaTime * timeWarpMultiplier;
      // Update orbital physics with sub-stepping (max 0.1 second physics steps)
      universe.update(warpedDeltaTime, 0.1);
    }

    // Update camera target to track selected object as it moves
    if (selectedObject.isValid())
    {
      glm::dvec3 objectPos = universe.getObjectPosition(selectedObject.object);
      camera.setTarget(glm::vec3(objectPos));
    }

    // Update window title with current time warp
    if (currentTime - lastTitleUpdateTime >= titleUpdateInterval)
    {
      std::string title = "Constellation Simulation";

      // Add pause/time warp status
      if (isPaused)
      {
        title += " [PAUSED]";
      }
      else
      {
        title += " [" + std::to_string((int)timeWarpMultiplier) + "x]";
      }

      glfwSetWindowTitle(window, title.c_str());
      lastTitleUpdateTime = currentTime;
    }

    /*-------------------- DRAW --------------------*/
    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Render main scene
    Satellite *selectedSat = selectedObject.asSatellite();
    renderer.render(universe, camera, state.windowWidth, state.windowHeight, vizState, selectedSat);

    // ========== ImGui UI Panel ==========
    if (selectedObject.isValid())
    {
      if (selectedObject.type == SelectedObject::Type::Satellite)
      {
        Satellite *sat = selectedObject.asSatellite();
        if (sat)
        {
          ImGui::Begin("Satellite Control Panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

          // ========== IDENTIFICATION ==========
          ImGui::SeparatorText("Satellite Information");
          ImGui::Text("Name: %s", sat->getName().c_str());
          ImGui::Text("Plane: %d  |  Index: %d", sat->getPlaneId(), sat->getIndexInPlane());

          // ========== ORBITAL STATE ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Orbital State");

          auto pos = sat->getPosition();
          double altitude = (glm::length(pos) - EARTH_RADIUS) / 1e3; // km
          ImGui::Text("Altitude: %.1f km", altitude);

          auto vel = sat->getVelocity();
          double speed = glm::length(vel) / 1e3; // km/s
          ImGui::Text("Velocity: %.3f km/s", speed);

          double orbitalPeriod = 2.0 * PI * sqrt(pow(glm::length(pos), 3) / (G * EARTH_MASS)) / 60.0; // minutes
          ImGui::Text("Orbital Period: %.1f min", orbitalPeriod);

          // ========== ATTITUDE STATE ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Attitude State");

          // Angular velocity (rotation rates)
          auto omega = sat->getAngularVelocity();
          double omegaMag = glm::length(omega) * 180.0 / PI; // Convert to deg/s
          ImGui::Text("Angular Velocity:");
          ImGui::Text("  X: %+.4f deg/s", omega.x * 180.0 / PI);
          ImGui::Text("  Y: %+.4f deg/s", omega.y * 180.0 / PI);
          ImGui::Text("  Z: %+.4f deg/s", omega.z * 180.0 / PI);
          ImGui::Text("  Magnitude: %.4f deg/s", omegaMag);

          // Quaternion
          auto quat = sat->getQuaternion();
          ImGui::Text("Quaternion: [%.3f, %.3f, %.3f, %.3f]", quat.w, quat.x, quat.y, quat.z);

          // ========== ATTITUDE CONTROL ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Attitude Control");

          // Control mode selection
          const char *controlModeNames[] = {
              "NONE (Tumbling)",
              "DETUMBLE",
              "NADIR POINTING",
              "SUN POINTING",
              "VELOCITY POINTING",
              "INERTIAL HOLD",
              "TARGET TRACKING"};
          int currentMode = (int)sat->getControlMode();
          if (ImGui::Combo("Control Mode", &currentMode, controlModeNames, IM_ARRAYSIZE(controlModeNames)))
          {
            sat->setControlMode((AttitudeControlMode)currentMode);
            std::cout << "Switched to control mode: " << controlModeNames[currentMode] << std::endl;
          }

          // Control algorithm selection
          const char *algorithmNames[] = {"PID", "LQR", "MPC"};
          int currentAlgo = (int)sat->getControlAlgorithm();
          if (ImGui::Combo("Control Algorithm", &currentAlgo, algorithmNames, IM_ARRAYSIZE(algorithmNames)))
          {
            sat->setControlAlgorithm((ControlAlgorithm)currentAlgo);
            std::cout << "Switched to control algorithm: " << algorithmNames[currentAlgo] << std::endl;
          }

          // ========== ACTUATORS STATUS ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Actuators");
          ImGui::Text("Reaction Wheels: ACTIVE");
          ImGui::Text("Magnetorquers: Available");
          ImGui::Text("CMGs: Available");

          // ========== POWER SYSTEM ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Power System");

          // Battery status
          double batteryPercent = sat->getBatteryPercentage();
          ImGui::Text("Battery: %.1f Wh / %.1f Wh", sat->getBatteryCharge(), sat->getBatteryCapacity());

          // Color-coded battery bar
          ImVec4 batteryColor;
          if (batteryPercent > 50.0)
            batteryColor = ImVec4(0.2f, 0.8f, 0.2f, 1.0f); // Green
          else if (batteryPercent > 20.0)
            batteryColor = ImVec4(0.9f, 0.7f, 0.2f, 1.0f); // Yellow
          else
            batteryColor = ImVec4(0.9f, 0.2f, 0.2f, 1.0f); // Red

          ImGui::PushStyleColor(ImGuiCol_PlotHistogram, batteryColor);
          ImGui::ProgressBar(sat->getBatteryPercentage() / 100.0f, ImVec2(0.0f, 0.0f),
                             (std::to_string((int)batteryPercent) + "%").c_str());
          ImGui::PopStyleColor();

          // Power generation and consumption
          double powerGen = sat->getPowerGeneration();
          double powerCons = sat->getPowerConsumption();
          double netPower = sat->getNetPower();

          ImGui::Text("Generation: %.1f W", powerGen);
          ImGui::SameLine(150);
          if (sat->isInEclipse())
          {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.9f, 1.0f), "[ECLIPSE]");
          }
          else
          {
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "[SUNLIT]");
          }

          ImGui::Text("Consumption: %.1f W", powerCons);

          ImGui::Text("Net Power: ");
          ImGui::SameLine();
          if (netPower > 0.0)
          {
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "+%.1f W (Charging)", netPower);
          }
          else
          {
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%.1f W (Draining)", netPower);
          }

          // ========== ADVANCED CONTROL TUNING ==========
          ImGui::Spacing();
          if (ImGui::CollapsingHeader("Advanced Control Tuning"))
          {
            if (sat->getControlAlgorithm() == ControlAlgorithm::PID)
            {
              ImGui::Text("PID Controller Gains:");
              if (ImGui::Button("Auto-Tune PID"))
              {
                sat->autoTunePID(20.0, 0.9); // 20s settling time, 0.9 damping
                std::cout << "PID auto-tuned for 20s settling time" << std::endl;
              }
              ImGui::SameLine();
              if (ImGui::Button("Reset Integral"))
              {
                sat->resetIntegralError();
              }
            }
            else if (sat->getControlAlgorithm() == ControlAlgorithm::LQR)
            {
              ImGui::Text("LQR Controller: Optimal state feedback");
            }
            else if (sat->getControlAlgorithm() == ControlAlgorithm::MPC)
            {
              ImGui::Text("MPC Controller: Predictive control");
            }

            ImGui::Spacing();
            ImGui::Text("Body Axes (Inertial Frame):");
            auto xAxis = sat->getBodyXAxis();
            auto yAxis = sat->getBodyYAxis();
            auto zAxis = sat->getBodyZAxis();
            ImGui::Text("  X: [%.3f, %.3f, %.3f]", xAxis.x, xAxis.y, xAxis.z);
            ImGui::Text("  Y: [%.3f, %.3f, %.3f]", yAxis.x, yAxis.y, yAxis.z);
            ImGui::Text("  Z: [%.3f, %.3f, %.3f]", zAxis.x, zAxis.y, zAxis.z);
          }

          // ========== PROPULSION ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Propulsion System");

          if (sat->hasPropellant())
          {
            double propellant = sat->getPropellantMass();
            double propellantPercent = sat->getPropellantFraction() * 100.0;
            ImGui::Text("Propellant: %.1f kg (%.1f%%)", propellant, propellantPercent);
            ImGui::ProgressBar(sat->getPropellantFraction(), ImVec2(0.0f, 0.0f));
          }
          else
          {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "No Propellant Remaining");
          }

          // ========== VISUALIZATION ==========
          ImGui::Spacing();
          ImGui::SeparatorText("Visualization (This Satellite)");

          auto &satViz = vizState.getOrCreate(sat);
          ImGui::Checkbox("Show Orbit Path", &satViz.showOrbitPath);
          ImGui::Checkbox("Show Footprint", &satViz.showFootprint);
          ImGui::Checkbox("Show Attitude Vector", &satViz.showAttitudeVector);

          ImGui::Spacing();
          ImGui::SeparatorText("Global Visualization");
          ImGui::Checkbox("Show All Orbit Paths", &vizState.showAllOrbitPaths);
          ImGui::Checkbox("Show All Attitude Vectors", &vizState.showAllAttitudeVectors);

          // ========== ACTIONS ==========
          ImGui::Spacing();
          ImGui::Separator();
          if (ImGui::Button("Deselect", ImVec2(120, 0)))
          {
            selectedObject.clear();
            camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
          }

          ImGui::End();
        }
      }
      else if (selectedObject.type == SelectedObject::Type::CelestialBody)
      {
        ImGui::Begin("Celestial Body", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::SeparatorText("Selected Object");
        CelestialBody *body = selectedObject.asCelestialBody();
        if (body)
        {
          auto pos = body->getPosition();
          ImGui::Text("Position: (%.0f, %.0f, %.0f) km",
                      pos.x / 1e3, pos.y / 1e3, pos.z / 1e3);
          ImGui::Text("Radius: %.0f km", body->getRadius() / 1e3);
        }

        ImGui::Spacing();
        if (ImGui::Button("Deselect"))
        {
          selectedObject.clear();
          camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
        }

        ImGui::End();
      }
    }
    else
    {
      // Show object list panel when no object selected
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

          // Identify which celestial body this is
          if (body == universe.getEarth())
            objectName = "Earth";
          else if (body == universe.getSun())
            objectName = "Sun";
          else if (body == universe.getMoon())
            objectName = "Moon";
          else
            objectName = "Celestial Body " + std::to_string(i);

          // Clickable button for each body
          if (ImGui::Selectable(objectName.c_str()))
          {
            selectedObject.type = SelectedObject::Type::CelestialBody;
            selectedObject.object = body.get();
            camera.setTarget(glm::vec3(body->getPosition()));
            std::cout << "Selected: " << objectName << std::endl;
          }
        }
      }

      // ========== SATELLITES ==========
      ImGui::Spacing();
      if (ImGui::CollapsingHeader("Satellites", ImGuiTreeNodeFlags_DefaultOpen))
      {
        auto &satellites = universe.getSatellites();

        // Group satellites by constellation/type for easier navigation
        std::map<int, std::vector<std::shared_ptr<Satellite>>> satellitesByPlane;
        for (const auto &sat : satellites)
        {
          satellitesByPlane[sat->getPlaneId()].push_back(sat);
        }

        // Display satellites grouped by plane
        for (auto &planePair : satellitesByPlane)
        {
          int planeId = planePair.first;
          auto &planeSats = planePair.second;

          // Create plane group name
          std::string planeLabel;
          if (planeId == -4)
            planeLabel = "GEO Constellation";
          else if (planeId == -3)
            planeLabel = "Reflect Constellation";
          else if (planeId == -2)
            planeLabel = "Molniya Constellation";
          else if (planeId >= 0 && !planeSats.empty())
          {
            // Determine constellation type from satellite name
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

              // Add altitude info for context
              double altitude = (glm::length(sat->getPosition()) - EARTH_RADIUS) / 1e3;
              satLabel += " (" + std::to_string((int)altitude) + " km)";

              if (ImGui::Selectable(satLabel.c_str()))
              {
                selectedObject.type = SelectedObject::Type::Satellite;
                selectedObject.object = sat.get();
                camera.setTarget(glm::vec3(sat->getPosition()));
                std::cout << "Selected: " << sat->getName() << std::endl;
              }
            }
            ImGui::TreePop();
          }
        }
      }

      ImGui::Spacing();
      ImGui::SeparatorText("Global Visualization");
      ImGui::Checkbox("Show All Orbit Paths", &vizState.showAllOrbitPaths);
      ImGui::Checkbox("Show All Attitude Vectors", &vizState.showAllAttitudeVectors);

      ImGui::End();
    }

    // Render ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Cleanup ImGui
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  // Cleanup GLFW
  glfwTerminate();

  return 0;
}