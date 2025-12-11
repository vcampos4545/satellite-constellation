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
#include "Renderer.h"

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
  int windowWidth;           // Current window width
  int windowHeight;          // Current window height
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
    // Render main scene
    renderer.render(universe, camera, state.windowWidth, state.windowHeight);
    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Cleanup
  glfwTerminate();

  return 0;
}