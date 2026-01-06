#include "Renderer.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Constants.h"
#include "OBJMesh.h"
#include "VisualizationState.h"

Renderer::Renderer()
    : sphereMesh(1.0f, 30, 30), // Unit sphere with 30x30 tessellation
      earthTextureLoaded(false),
      moonTextureLoaded(false),
      sunTextureLoaded(false),
      starsTextureLoaded(false),
      starlinkMeshLoaded(false),
      initialized(false)
{
}

Renderer::~Renderer()
{
  // Meshes, shaders, and textures clean themselves up automatically
}

bool Renderer::initialize()
{
  if (initialized)
  {
    return true;
  }

  // Load shaders
  sphereShader = std::make_unique<Shader>("shaders/sphere.vert", "shaders/sphere.frag");
  lineShader = std::make_unique<Shader>("shaders/line.vert", "shaders/line.frag");

  // Load Earth texture
  earthTextureLoaded = earthTexture.load("textures/earth.jpg");
  if (!earthTextureLoaded)
  {
    std::cout << "Note: Earth texture not found. Using solid color. Place an Earth texture at 'textures/earth.jpg'" << std::endl;
  }

  // Load Moon texture
  moonTextureLoaded = moonTexture.load("textures/moon.jpg");
  if (!moonTextureLoaded)
  {
    std::cout << "Note: Moon texture not found. Using solid color. Place a Moon texture at 'textures/moon.jpg'" << std::endl;
  }

  // Load Sun texture
  sunTextureLoaded = sunTexture.load("textures/sun.jpg");
  if (!sunTextureLoaded)
  {
    std::cout << "Note: Sun texture not found. Using solid color. Place a Sun texture at 'textures/sun.jpg'" << std::endl;
  }

  // Load Stars texture for background
  starsTextureLoaded = starsTexture.load("textures/stars.jpg");
  if (!starsTextureLoaded)
  {
    std::cout << "Note: Stars texture not found. Using black background. Place a stars texture at 'textures/stars.jpg'" << std::endl;
  }

  // Load Starlink satellite mesh
  starlinkMeshLoaded = starlinkMesh.load("models/starlink/starlink.obj");
  if (!starlinkMeshLoaded)
  {
    std::cout << "Note: Starlink mesh not found. Using simple geometry'" << std::endl;
  }

  // Load Starlink satellite mesh
  cubesat1UMeshLoaded = cubesat1UMesh.load("models/cubesat1U/cubesat1U.obj");
  if (!cubesat1UMeshLoaded)
  {
    std::cout << "Note: Cubesat1U mesh not found. Using simple geometry'" << std::endl;
  }

  // Load Starlink satellite mesh
  cubesat2UMeshLoaded = cubesat2UMesh.load("models/cubesat2U/cubesat2U.obj");
  if (!cubesat2UMeshLoaded)
  {
    std::cout << "Note: Cubesat2U mesh not found. Using simple geometry'" << std::endl;
  }

  initialized = true;
  return true;
}

void Renderer::render(const Universe &universe, const Camera &camera, int windowWidth, int windowHeight,
                      const VisualizationState &vizState, const Satellite *selectedSatellite)
{
  if (!initialized)
  {
    std::cerr << "Renderer not initialized! Call initialize() first." << std::endl;
    return;
  }

  // Clear the screen
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Render star background first (before other objects)
  renderStarBackground(camera);

  // Set up shader with view and projection matrices
  sphereShader->use();
  glm::mat4 view = camera.getViewMatrix();
  glm::mat4 projection = camera.getProjectionMatrix();
  sphereShader->setMat4("view", view);
  sphereShader->setMat4("projection", projection);

  // Set camera position for lighting
  sphereShader->setVec3("viewPos", camera.getPosition());

  // Calculate light direction (from Earth to Sun)
  glm::vec3 sunPos = glm::vec3(universe.getSun()->getPosition());
  glm::vec3 earthPos = glm::vec3(universe.getEarth()->getPosition());
  glm::vec3 lightDir = glm::normalize(sunPos - earthPos);
  sphereShader->setVec3("lightDir", lightDir);

  // Render celestial bodies
  renderCelestialBody(universe.getEarth(), true, false, false);
  renderCelestialBody(universe.getMoon(), false, true, false);
  renderCelestialBody(universe.getSun(), false, false, true);

  // Render ground stations
  renderGroundStations(universe.getGroundStations());

  // Set up line shader for line-based rendering (needed for orbits, footprints, attitude vectors)
  lineShader->use();
  lineShader->setMat4("view", view);
  lineShader->setMat4("projection", projection);

  // Render satellites with all visualization options
  renderSatellites(universe.getSatellites(), vizState, selectedSatellite);

  // Render coordinate axis overlay
  renderCoordinateAxis(camera, windowWidth, windowHeight);
}

void Renderer::renderStarBackground(const Camera &camera)
{
  if (!starsTextureLoaded)
    return;

  // Disable depth writing so stars are always in the background
  glDepthMask(GL_FALSE);

  // Create view matrix with only rotation (no translation)
  glm::mat4 view = glm::mat4(glm::mat3(camera.getViewMatrix()));

  // Use sphere shader
  sphereShader->use();
  sphereShader->setMat4("view", view);
  sphereShader->setMat4("projection", camera.getProjectionMatrix());

  // Create model matrix for background sphere
  // Make it very large so it encompasses everything
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::scale(model, glm::vec3(1e11f)); // 100 billion meters radius

  sphereShader->setMat4("model", model);
  sphereShader->setVec3("objectColor", glm::vec3(0.25f, 0.25f, 0.25f)); // Dimmed significantly

  // Bind stars texture
  starsTexture.bind(0);
  sphereShader->setInt("textureSampler", 0);
  sphereShader->setBool("useTexture", true);

  // Make stars self-illuminated (no lighting applied)
  sphereShader->setBool("isEmissive", true);

  // Draw the background sphere
  sphereMesh.draw();

  // Re-enable depth writing
  glDepthMask(GL_TRUE);
}

void Renderer::renderCelestialBody(const std::shared_ptr<CelestialBody> &body, bool isEarth, bool isMoon, bool isSun)
{
  if (!body)
    return;

  sphereShader->use();

  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, glm::vec3(body->getPosition()));

  // Apply rotation for Earth's spin
  if (isEarth)
  {
    // Apply time-based rotation for Earth's spin around Z-axis
    model = glm::rotate(model, body->getRotation(), body->getRotationAxis());
  }

  model = glm::scale(model, glm::vec3(body->getRadius()));
  sphereShader->setMat4("model", model);
  sphereShader->setVec3("objectColor", body->getColor());

  // Set lighting properties based on body type
  if (isSun)
  {
    // Sun is self-illuminated and very bright
    sphereShader->setFloat("ambientStrength", 1.0f);
    sphereShader->setBool("isEmissive", true);

    if (sunTextureLoaded)
    {
      sunTexture.bind(0);
      sphereShader->setInt("textureSampler", 0);
      sphereShader->setBool("useTexture", true);
    }
    else
    {
      sphereShader->setBool("useTexture", false);
    }
  }
  else if (isEarth)
  {
    // Earth has higher ambient to make dark side visible
    sphereShader->setFloat("ambientStrength", 0.35f);
    sphereShader->setBool("isEmissive", false);

    // Use texture if available
    if (earthTextureLoaded)
    {
      earthTexture.bind(0);
      sphereShader->setInt("textureSampler", 0);
      sphereShader->setBool("useTexture", true);
    }
    else
    {
      sphereShader->setBool("useTexture", false);
    }
  }
  else if (isMoon)
  {
    // Moon has slightly higher ambient for visibility
    sphereShader->setFloat("ambientStrength", 0.2f);
    sphereShader->setBool("isEmissive", false);

    // Use texture if available
    if (moonTextureLoaded)
    {
      moonTexture.bind(0);
      sphereShader->setInt("textureSampler", 0);
      sphereShader->setBool("useTexture", true);
    }
    else
    {
      sphereShader->setBool("useTexture", false);
    }
  }

  sphereMesh.draw();
}

void Renderer::renderSatellites(const std::vector<std::shared_ptr<Satellite>> &satellites,
                                const VisualizationState &vizState,
                                const Satellite *selectedSatellite)
{
  // Single loop through all satellites, rendering each component as needed
  for (const auto &satellite : satellites)
  {
    // Render historical orbit path
    if (vizState.shouldDrawOrbit(satellite.get()))
    {
      renderSatelliteOrbitPath(satellite, vizState);
    }

    // Render predicted future orbit (only for selected satellite)
    if (selectedSatellite && satellite.get() == selectedSatellite)
    {
      renderSatellitePredictedOrbit(satellite, selectedSatellite);
    }

    // Render satellite 3D geometry (always rendered)
    renderSatelliteGeometry(satellite);

    // Render ground footprint circle
    if (vizState.shouldDrawFootprint(satellite.get()))
    {
      renderSatelliteFootprint(satellite, vizState);
    }

    // Render attitude pointing vector
    if (vizState.shouldDrawAttitudeVector(satellite.get()))
    {
      renderSatelliteAttitudeVector(satellite, vizState);
    }
  }
}

// ========================================
// SATELLITE RENDER HELPER FUNCTIONS
// ========================================

void Renderer::renderSatelliteOrbitPath(const std::shared_ptr<Satellite> &satellite,
                                        const VisualizationState &vizState)
{
  const auto &orbitPath = satellite->getOrbitPath();
  if (orbitPath.size() < 2)
    return;

  // Convert dvec3 to vec3 for rendering
  std::vector<glm::vec3> pathVertices;
  pathVertices.reserve(orbitPath.size());
  for (const auto &pos : orbitPath)
  {
    pathVertices.push_back(glm::vec3(pos));
  }

  lineRenderer.setVertices(pathVertices);

  // Setup line rendering
  glLineWidth(10.0f);
  lineShader->use();
  glm::vec3 orbitColor = satellite->getColor();
  lineShader->setVec3("lineColor", orbitColor);
  lineRenderer.draw();
  glLineWidth(2.0f); // Reset
}

void Renderer::renderSatellitePredictedOrbit(const std::shared_ptr<Satellite> &satellite,
                                             const Satellite *selectedSatellite)
{
  const auto &predictedPath = satellite->getPredictedOrbit();
  if (predictedPath.size() < 2)
    return;

  // Convert dvec3 to vec3 for rendering
  std::vector<glm::vec3> predVertices;
  predVertices.reserve(predictedPath.size());
  for (const auto &pos : predictedPath)
  {
    predVertices.push_back(glm::vec3(pos));
  }

  lineRenderer.setVertices(predVertices);

  // Setup line rendering (cyan color for predicted orbit)
  glLineWidth(10.0f);
  lineShader->use();
  glm::vec3 predictedColor = glm::vec3(0.3f, 0.8f, 1.0f); // Cyan
  lineShader->setVec3("lineColor", predictedColor);
  lineRenderer.draw();
  glLineWidth(2.0f); // Reset
}

void Renderer::renderSatelliteGeometry(const std::shared_ptr<Satellite> &satellite)
{
  glm::vec3 satPos = glm::vec3(satellite->getPosition());

  // Get satellite attitude (quaternion -> rotation matrix)
  glm::quat attitudeQuat = glm::quat(satellite->getQuaternion());
  glm::mat4 attitudeMatrix = glm::mat4_cast(attitudeQuat);

  // Setup shader for satellite rendering
  sphereShader->use();
  sphereShader->setBool("useTexture", false);
  sphereShader->setFloat("ambientStrength", 1.0f);
  sphereShader->setBool("isEmissive", true); // Satellites are self-illuminated

  // Check satellite type for specialized mesh
  bool isStarlink = satellite->getName().find("Starlink") != std::string::npos;
  bool isCubesat1U = satellite->getName().find("Cubesat1U") != std::string::npos;
  bool isCubesat2U = satellite->getName().find("Cubesat2U") != std::string::npos;

  if ((isStarlink && starlinkMeshLoaded) || (isCubesat1U && cubesat1UMeshLoaded) || (isCubesat2U && cubesat2UMeshLoaded))
  {
    // Render 3D model with materials
    glm::mat4 meshModel = glm::mat4(1.0f);
    meshModel = glm::translate(meshModel, satPos);
    meshModel = meshModel * attitudeMatrix;
    meshModel = glm::scale(meshModel, glm::vec3(0.8e4f)); // ~8km scale
    sphereShader->setMat4("model", meshModel);

    if (isStarlink)
      starlinkMesh.draw(*sphereShader);
    else if (isCubesat1U)
      cubesat1UMesh.draw(*sphereShader);
    else if (isCubesat2U)
      cubesat2UMesh.draw(*sphereShader);
  }
  else
  {
    // Fallback: Simple geometry (cube + solar panels)
    glm::vec3 brightColor = satellite->getColor() * 2.0f;
    sphereShader->setVec3("objectColor", brightColor);

    // Central body
    glm::mat4 bodyModel = glm::mat4(1.0f);
    bodyModel = glm::translate(bodyModel, satPos);
    bodyModel = bodyModel * attitudeMatrix;
    bodyModel = glm::scale(bodyModel, glm::vec3(1.6e4f, 2.4e4f, 1.6e4f));
    sphereShader->setMat4("model", bodyModel);
    cubeMesh.draw();

    // Solar panels (blue color)
    glm::vec3 panelColor = glm::vec3(0.4f, 0.6f, 1.0f);
    sphereShader->setVec3("objectColor", panelColor);

    // Left panel
    glm::mat4 leftPanelModel = glm::mat4(1.0f);
    leftPanelModel = glm::translate(leftPanelModel, satPos);
    leftPanelModel = leftPanelModel * attitudeMatrix;
    leftPanelModel = glm::translate(leftPanelModel, glm::vec3(-3.0e4f, 0.0f, 0.0f));
    leftPanelModel = glm::scale(leftPanelModel, glm::vec3(4.0e4f, 3.6e4f, 0.4e4f));
    sphereShader->setMat4("model", leftPanelModel);
    cubeMesh.draw();

    // Right panel
    glm::mat4 rightPanelModel = glm::mat4(1.0f);
    rightPanelModel = glm::translate(rightPanelModel, satPos);
    rightPanelModel = rightPanelModel * attitudeMatrix;
    rightPanelModel = glm::translate(rightPanelModel, glm::vec3(3.0e4f, 0.0f, 0.0f));
    rightPanelModel = glm::scale(rightPanelModel, glm::vec3(4.0e4f, 3.6e4f, 0.4e4f));
    sphereShader->setMat4("model", rightPanelModel);
    cubeMesh.draw();
  }
}

void Renderer::renderSatelliteFootprint(const std::shared_ptr<Satellite> &satellite,
                                        const VisualizationState &vizState)
{
  const auto &footprintCircle = satellite->getFootprintCircle();
  if (footprintCircle.size() < 2)
    return;

  // Convert dvec3 to vec3 for rendering
  std::vector<glm::vec3> footprintVertices;
  footprintVertices.reserve(footprintCircle.size());
  for (const auto &pos : footprintCircle)
  {
    footprintVertices.push_back(glm::vec3(pos));
  }

  lineRenderer.setVertices(footprintVertices);

  // Setup line rendering (yellow-green for footprint)
  glLineWidth(2.0f);
  lineShader->use();
  glm::vec3 footprintColor = glm::vec3(0.8f, 1.0f, 0.3f);
  lineShader->setVec3("lineColor", footprintColor);
  lineRenderer.draw();
}

void Renderer::renderSatelliteAttitudeVector(const std::shared_ptr<Satellite> &satellite,
                                             const VisualizationState &vizState)
{
  // Get satellite position and pointing direction (Z-axis in body frame)
  glm::dvec3 satPos = satellite->getPosition();
  glm::dvec3 zAxis = satellite->getBodyZAxis();

  // Scale arrow to be visible (500 km long)
  double arrowLength = 500e3;
  glm::dvec3 arrowEnd = satPos + zAxis * arrowLength;

  // Create arrow vertices
  std::vector<glm::vec3> attitudeArrow = {
      glm::vec3(satPos),
      glm::vec3(arrowEnd)};

  lineRenderer.setVertices(attitudeArrow);

  // Setup line rendering (red for attitude vector)
  glLineWidth(3.0f);
  lineShader->use();
  glm::vec3 attitudeColor = glm::vec3(1.0f, 0.0f, 0.0f); // Red
  lineShader->setVec3("lineColor", attitudeColor);
  lineRenderer.draw();
  glLineWidth(2.0f); // Reset
}

void Renderer::renderGroundStations(const std::vector<std::shared_ptr<GroundStation>> &groundStations)
{
  sphereShader->use();
  sphereShader->setBool("useTexture", false);
  sphereShader->setFloat("ambientStrength", 0.5f); // Higher ambient for visibility
  sphereShader->setBool("isEmissive", false);      // Ground stations use normal lighting

  for (const auto &groundStation : groundStations)
  {
    glm::vec3 stationPos = glm::vec3(groundStation->getPosition());

    // Small sphere for ground station (50 km radius)
    glm::mat4 stationModel = glm::mat4(1.0f);
    stationModel = glm::translate(stationModel, stationPos);
    stationModel = glm::scale(stationModel, glm::vec3(5.0e4f)); // 50 km radius
    sphereShader->setMat4("model", stationModel);

    // Use bright red color for ground stations
    glm::vec3 stationColor = glm::vec3(1.0f, 0.2f, 0.2f); // Bright red
    sphereShader->setVec3("objectColor", stationColor);
    sphereMesh.draw();
  }
}

void Renderer::renderCoordinateAxis(const Camera &camera, int windowWidth, int windowHeight)
{
  // Save current viewport
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  // Set viewport for axis display in bottom left corner
  int axisSize = 350; // Size of the axis display area (200x200 pixels)
  int margin = 10;    // Margin from the corner
  glViewport(margin, margin, axisSize, axisSize);

  // Disable depth test for overlay rendering
  glDisable(GL_DEPTH_TEST);

  // Create orthographic projection for the axis
  glm::mat4 projection = glm::ortho(-1.5f, 1.5f, -1.5f, 1.5f, -10.0f, 10.0f);

  // Create view matrix that only rotates (no translation)
  // Extract rotation from camera view matrix
  glm::mat4 cameraView = camera.getViewMatrix();
  glm::mat4 axisView = glm::mat4(glm::mat3(cameraView)); // Remove translation component

  // Set up shader
  lineShader->use();
  lineShader->setMat4("view", axisView);
  lineShader->setMat4("projection", projection);

  // Increase line width for better visibility
  glLineWidth(10.0f);

  // Draw X axis (red)
  std::vector<glm::vec3> xAxis = {glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)};
  lineRenderer.setVertices(xAxis);
  lineShader->setVec3("lineColor", glm::vec3(1.0f, 0.0f, 0.0f)); // Red
  lineRenderer.draw();

  // Draw Y axis (green)
  std::vector<glm::vec3> yAxis = {glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)};
  lineRenderer.setVertices(yAxis);
  lineShader->setVec3("lineColor", glm::vec3(0.0f, 1.0f, 0.0f)); // Green
  lineRenderer.draw();

  // Draw Z axis (blue)
  std::vector<glm::vec3> zAxis = {glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)};
  lineRenderer.setVertices(zAxis);
  lineShader->setVec3("lineColor", glm::vec3(0.0f, 0.0f, 1.0f)); // Blue
  lineRenderer.draw();

  // Reset line width
  glLineWidth(1.0f);

  // Re-enable depth test
  glEnable(GL_DEPTH_TEST);

  // Restore original viewport
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}
