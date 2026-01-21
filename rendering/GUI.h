/*
Main GUI class.

Set up and opengl API functions for the simulation
*/

#ifndef GUI_H
#define GUI_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <memory>
#include "Camera.h"
#include "Renderer.h"
#include "VisualizationState.h"

// Forward declaration
class Simulation;

class GUI
{
public:
  GUI(int screenWidth, int screenHeight, Simulation *simulation);
  ~GUI();

  // Main render function
  void render();

  // Window control
  bool shouldClose() const;

private:
  // Reference to simulation (for callbacks to control sim)
  Simulation *m_simulation;

  // OpenGL/GLFW state
  GLFWwindow *m_window;
  int m_windowWidth;
  int m_windowHeight;

  // Rendering components
  Camera m_camera;
  std::unique_ptr<Renderer> m_renderer;

  // UI state
  VisualizationState m_vizState;
  SelectedObject m_selectedObject;

  // Input state
  bool m_isDragging;
  double m_lastMouseX;
  double m_lastMouseY;
  double m_clickMouseX;
  double m_clickMouseY;
  float m_currentTheta;
  float m_currentPhi;

  // Setup functions
  void initWindow(int width, int height);
  void initCamera();
  void initRenderer();
  void initImGui();
  void setupCallbacks();

  // Rendering functions
  void renderScene();
  void renderMenuBar();
  void renderUI();

  // Static callback functions (GLFW requires static functions)
  static void scrollCallbackStatic(GLFWwindow *window, double xoffset, double yoffset);
  static void mouseButtonCallbackStatic(GLFWwindow *window, int button, int action, int mods);
  static void cursorPosCallbackStatic(GLFWwindow *window, double xpos, double ypos);
  static void framebufferSizeCallbackStatic(GLFWwindow *window, int width, int height);
  static void keyCallbackStatic(GLFWwindow *window, int key, int scancode, int action, int mods);
  static void charCallbackStatic(GLFWwindow *window, unsigned int c);

  // Instance callback functions (called by static callbacks)
  void scrollCallback(double xoffset, double yoffset);
  void mouseButtonCallback(int button, int action, int mods);
  void cursorPosCallback(double xpos, double ypos);
  void framebufferSizeCallback(int width, int height);
  void keyCallback(int key, int scancode, int action, int mods);
  void charCallback(unsigned int c);
};

#endif // GUI_H