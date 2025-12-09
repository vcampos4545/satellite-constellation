/*
Satellite Constelation

This simulation aims to display acurately the earth and a constelation of satelites.

These are simulated space based solar power satellites, beaming energy to earth at various locations

*/

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Constants.h"
#include "Universe.h"
#include "Camera.h"
#include "Sphere.h"
#include "Cube.h"
#include "Shader.h"
#include "Texture.h"
#include "LineRenderer.h"

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
  float currentTheta;
  float currentPhi;
  float *timeWarpMultiplier; // Pointer to time warp multiplier
  bool *isPaused;            // Pointer to pause state
};

// Mouse scroll callback for zooming
void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (state && state->camera)
  {
    // Zoom speed depends on current distance
    float zoomSpeed = state->camera->getDistance() * 0.1f;
    state->camera->adjustDistance(-yoffset * zoomSpeed);
  }
}

// Mouse button callback for starting/stopping pan
void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
  State *state = static_cast<State *>(glfwGetWindowUserPointer(window));
  if (state)
  {
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
      if (action == GLFW_PRESS)
      {
        state->isDragging = true;
        glfwGetCursorPos(window, &state->lastMouseX, &state->lastMouseY);
      }
      else if (action == GLFW_RELEASE)
      {
        state->isDragging = false;
      }
    }
  }
}

// Cursor position callback for panning
void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
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
  else if (key == GLFW_KEY_S) // TEMPORARY: Press s to focus on random satellite
  {
    Satellite sat = *state->universe->getSatellites()[0];
    state->camera->setTarget(sat.getPosition());
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
  Camera camera(45.0f, (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 1e5f, 2e11f); // Far plane extended to 200 million km to see the sun
  camera.setTarget(glm::vec3(0.0f, 0.0f, 0.0f));
  camera.setDistance(1.5e7f); // 15,000 km from Earth - good view of LEO constellation

  // Time control state
  float timeWarpMultiplier = 1.0f; // Start at 1x speed
  bool isPaused = false;

  // Initialize input state
  State state;
  state.camera = &camera;
  state.universe = &universe;
  state.isDragging = false;
  state.lastMouseX = 0.0;
  state.lastMouseY = 0.0;
  state.currentTheta = 0.0f;
  state.currentPhi = glm::pi<float>() / 4.0f; // Match initial camera phi
  state.timeWarpMultiplier = &timeWarpMultiplier;
  state.isPaused = &isPaused;

  // Set up input callbacks
  glfwSetWindowUserPointer(window, &state);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetCursorPosCallback(window, cursorPosCallback);
  glfwSetKeyCallback(window, keyCallback);

  // Load shaders
  Shader sphereShader("shaders/sphere.vert", "shaders/sphere.frag");
  Shader lineShader("shaders/line.vert", "shaders/line.frag");

  // Create sphere mesh for rendering (unit sphere)
  Sphere sphereMesh(1.0f, 30, 30);

  // Create cube mesh for rendering satellites
  Cube cubeMesh;

  // Create line renderer for orbit paths
  LineRenderer lineRenderer;

  // Load Earth texture
  Texture earthTexture;
  bool hasEarthTexture = earthTexture.load("textures/earth.jpg");
  if (!hasEarthTexture)
  {
    std::cout << "Note: Earth texture not found. Using solid color. Place an Earth texture at 'textures/earth.jpg'" << std::endl;
  }

  // Load Moon texture
  Texture moonTexture;
  bool hasMoonTexture = moonTexture.load("textures/moon.jpg");
  if (!hasMoonTexture)
  {
    std::cout << "Note: Moon texture not found. Using solid color. Place a Moon texture at 'textures/moon.jpg'" << std::endl;
  }

  // Print controls
  std::cout << "\n=== CONTROLS ===" << std::endl;
  std::cout << "Mouse Drag: Pan camera around Earth" << std::endl;
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

  while (!glfwWindowShouldClose(window))
  {
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

    // Update window title with current time warp
    if (currentTime - lastTitleUpdateTime >= titleUpdateInterval)
    {
      std::string title = "Constellation Simulation";
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

    // Clear the screen
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use shader
    sphereShader.use();

    // Set view and projection matrices
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix();
    sphereShader.setMat4("view", view);
    sphereShader.setMat4("projection", projection);

    // Set camera position for lighting
    sphereShader.setVec3("viewPos", camera.getPosition());

    // Calculate light direction (from Earth to Sun)
    glm::vec3 sunPos = glm::vec3(universe.getSun()->getPosition());
    glm::vec3 earthPos = glm::vec3(universe.getEarth()->getPosition());
    glm::vec3 lightDir = glm::normalize(sunPos - earthPos);
    sphereShader.setVec3("lightDir", lightDir);

    // Render Earth
    auto earth = universe.getEarth();
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(earth->getPosition()));

    // Apply base rotation to align texture with coordinate system
    // Adjust this value if cities don't line up with the texture
    // 90° rotation aligns equirectangular texture's center (0° lon) with +X axis
    model = glm::rotate(model, glm::radians(90.0f), earth->getRotationAxis());

    // Apply time-based rotation for Earth's spin around its tilted axis
    model = glm::rotate(model, earth->getRotation(), earth->getRotationAxis());

    model = glm::scale(model, glm::vec3(earth->getRadius()));
    sphereShader.setMat4("model", model);
    sphereShader.setVec3("objectColor", earth->getColor());

    // Use texture if available
    if (hasEarthTexture)
    {
      earthTexture.bind(0);
      sphereShader.setInt("textureSampler", 0);
      sphereShader.setBool("useTexture", true);
    }
    else
    {
      sphereShader.setBool("useTexture", false);
    }

    sphereMesh.draw();

    // Render Moon
    auto moon = universe.getMoon();
    if (moon)
    {
      glm::mat4 moonModel = glm::mat4(1.0f);
      moonModel = glm::translate(moonModel, glm::vec3(moon->getPosition()));
      moonModel = glm::scale(moonModel, glm::vec3(moon->getRadius()));
      sphereShader.setMat4("model", moonModel);
      sphereShader.setVec3("objectColor", moon->getColor());

      // Use texture if available
      if (hasMoonTexture)
      {
        moonTexture.bind(0);
        sphereShader.setInt("textureSampler", 0);
        sphereShader.setBool("useTexture", true);
      }
      else
      {
        sphereShader.setBool("useTexture", false);
      }

      sphereMesh.draw();
    }

    // Render Sun (make it bright and self-lit)
    auto sun = universe.getSun();
    if (sun)
    {
      glm::mat4 sunModel = glm::mat4(1.0f);
      sunModel = glm::translate(sunModel, glm::vec3(sun->getPosition()));
      sunModel = glm::scale(sunModel, glm::vec3(sun->getRadius()));
      sphereShader.setMat4("model", sunModel);
      // Make the sun very bright (emissive-like by using high values)
      glm::vec3 sunColor = sun->getColor() * 3.0f; // Extra bright
      sphereShader.setVec3("objectColor", sunColor);
      sphereShader.setBool("useTexture", false);
      sphereMesh.draw();
    }

    // Render satellites with body + solar panels
    sphereShader.setBool("useTexture", false);
    for (const auto &satellite : universe.getSatellites())
    {
      glm::vec3 satPos = glm::vec3(satellite->getPosition());
      glm::vec3 brightColor = satellite->getColor() * 1.5f; // Make satellites brighter
      sphereShader.setVec3("objectColor", brightColor);

      // Get satellite attitude (quaternion -> rotation matrix)
      glm::quat attitudeQuat = glm::quat(satellite->getQuaternion()); // Convert dquat to quat
      glm::mat4 attitudeMatrix = glm::mat4_cast(attitudeQuat);

      // Render central body (elongated box)
      // Apply: Translation -> Rotation (attitude) -> Scale
      glm::mat4 bodyModel = glm::mat4(1.0f);
      bodyModel = glm::translate(bodyModel, satPos);
      bodyModel = bodyModel * attitudeMatrix;                               // Apply attitude rotation
      bodyModel = glm::scale(bodyModel, glm::vec3(8.0e4f, 1.2e5f, 8.0e4f)); // 80km x 120km x 80km
      sphereShader.setMat4("model", bodyModel);
      cubeMesh.draw();

      // Render left solar panel (thin flat box)
      // Solar panels extend along X-axis in body frame
      glm::mat4 leftPanelModel = glm::mat4(1.0f);
      leftPanelModel = glm::translate(leftPanelModel, satPos);
      leftPanelModel = leftPanelModel * attitudeMatrix;                                // Apply attitude rotation
      leftPanelModel = glm::translate(leftPanelModel, glm::vec3(-1.5e5f, 0.0f, 0.0f)); // Offset to the left (in body frame)
      leftPanelModel = glm::scale(leftPanelModel, glm::vec3(2.0e5f, 1.8e5f, 0.2e5f));  // 200km x 180km x 20km (thin)
      sphereShader.setMat4("model", leftPanelModel);

      // Make solar panels slightly darker (blueish tint for solar cells)
      glm::vec3 panelColor = glm::vec3(0.2f, 0.3f, 0.5f);
      sphereShader.setVec3("objectColor", panelColor);
      cubeMesh.draw();

      // Render right solar panel
      glm::mat4 rightPanelModel = glm::mat4(1.0f);
      rightPanelModel = glm::translate(rightPanelModel, satPos);
      rightPanelModel = rightPanelModel * attitudeMatrix;                               // Apply attitude rotation
      rightPanelModel = glm::translate(rightPanelModel, glm::vec3(1.5e5f, 0.0f, 0.0f)); // Offset to the right (in body frame)
      rightPanelModel = glm::scale(rightPanelModel, glm::vec3(2.0e5f, 1.8e5f, 0.2e5f)); // 200km x 180km x 20km (thin)
      sphereShader.setMat4("model", rightPanelModel);
      sphereShader.setVec3("objectColor", panelColor);
      cubeMesh.draw();
    }

    // Render ground stations as small dots
    sphereShader.use();
    sphereShader.setBool("useTexture", false);
    for (const auto &groundStation : universe.getGroundStations())
    {
      glm::vec3 stationPos = glm::vec3(groundStation->getPosition());

      // Small sphere for ground station (50 km radius)
      glm::mat4 stationModel = glm::mat4(1.0f);
      stationModel = glm::translate(stationModel, stationPos);
      stationModel = glm::scale(stationModel, glm::vec3(5.0e4f)); // 50 km radius
      sphereShader.setMat4("model", stationModel);

      // Use bright red color for ground stations
      glm::vec3 stationColor = glm::vec3(1.0f, 0.2f, 0.2f); // Bright red
      sphereShader.setVec3("objectColor", stationColor);
      sphereMesh.draw();
    }

    // Render power beams from ground stations to all visible satellites
    glLineWidth(2.0f);
    lineShader.use();
    lineShader.setMat4("view", view);
    lineShader.setMat4("projection", projection);

    for (const auto &groundStation : universe.getGroundStations())
    {
      // Draw beams to all visible satellites
      for (const auto &satellite : groundStation->getVisibleSatellites())
      {
        // Draw power beam from ground station to satellite
        std::vector<glm::vec3> beamVertices;
        beamVertices.push_back(glm::vec3(groundStation->getPosition()));
        beamVertices.push_back(glm::vec3(satellite->getPosition()));

        lineRenderer.setVertices(beamVertices);

        // Use bright yellow/orange color for energy beam
        glm::vec3 beamColor = glm::vec3(1.0f, 0.8f, 0.1f); // Bright yellow-orange
        lineShader.setVec3("lineColor", beamColor);
        lineRenderer.draw();
      }
    }

    // Render orbit paths with improved visuals (only one per orbital plane)
    for (const auto &satellite : universe.getSatellites())
    {
      // Only draw orbit for the first satellite in each plane to reduce clutter
      if (!satellite->shouldDrawOrbit())
        continue;

      const auto &orbitPath = satellite->getOrbitPath();
      if (orbitPath.size() >= 2)
      {
        // Convert dvec3 to vec3 for rendering
        std::vector<glm::vec3> pathVertices;
        pathVertices.reserve(orbitPath.size());
        for (const auto &pos : orbitPath)
        {
          pathVertices.push_back(glm::vec3(pos));
        }

        lineRenderer.setVertices(pathVertices);

        // Make orbit lines slightly dimmer than satellites
        glm::vec3 orbitColor = satellite->getColor() * 0.6f;
        lineShader.setVec3("lineColor", orbitColor);
        lineRenderer.draw();
      }
    }

    // Render footprint circles on Earth's surface
    for (const auto &satellite : universe.getSatellites())
    {
      const auto &footprintCircle = satellite->getFootprintCircle();
      if (footprintCircle.size() >= 2)
      {
        // Convert dvec3 to vec3 for rendering
        std::vector<glm::vec3> footprintVertices;
        footprintVertices.reserve(footprintCircle.size());
        for (const auto &pos : footprintCircle)
        {
          footprintVertices.push_back(glm::vec3(pos));
        }

        lineRenderer.setVertices(footprintVertices);

        // Use a semi-transparent yellow/green color for footprint circles
        glm::vec3 footprintColor = glm::vec3(0.8f, 1.0f, 0.3f); // Yellow-green
        lineShader.setVec3("lineColor", footprintColor);
        lineRenderer.draw();
      }
    }

    // Render satellite attitude vectors (pointing direction)
    glLineWidth(3.0f); // Thicker line for attitude arrow
    for (const auto &satellite : universe.getSatellites())
    {
      // Get satellite position and pointing direction (Z-axis in body frame)
      glm::dvec3 satPos = satellite->getPosition();
      glm::dvec3 zAxis = satellite->getBodyZAxis();

      // Scale arrow to be visible (e.g., 500 km long)
      double arrowLength = 500e3; // 500 km
      glm::dvec3 arrowEnd = satPos + zAxis * arrowLength;

      // Create arrow vertices
      std::vector<glm::vec3> attitudeArrow = {
          glm::vec3(satPos),
          glm::vec3(arrowEnd)};

      lineRenderer.setVertices(attitudeArrow);

      // Use bright red for attitude vector
      glm::vec3 attitudeColor = glm::vec3(1.0f, 0.0f, 0.0f); // Red
      lineShader.setVec3("lineColor", attitudeColor);
      lineRenderer.draw();
    }
    glLineWidth(2.0f); // Reset to normal line width

    glLineWidth(1.0f); // Reset line width

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Cleanup
  glfwTerminate();

  return 0;
}