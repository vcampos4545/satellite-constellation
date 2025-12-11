#include "Renderer.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Constants.h"
#include "OBJMesh.h"

Renderer::Renderer()
    : sphereMesh(1.0f, 30, 30), // Unit sphere with 30x30 tessellation
      earthTextureLoaded(false),
      moonTextureLoaded(false),
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

  // Load Stars texture for background
  starsTextureLoaded = starsTexture.load("textures/stars.jpg");
  if (!starsTextureLoaded)
  {
    std::cout << "Note: Stars texture not found. Using black background. Place a stars texture at 'textures/stars.jpg'" << std::endl;
  }

  // Load Starlink satellite mesh
  starlinkMeshLoaded = starlinkMesh.load("models/starlink.obj");
  if (!starlinkMeshLoaded)
  {
    std::cout << "Note: Starlink mesh not found. Using simple geometry. Place Starlink OBJ at 'models/starlink.obj'" << std::endl;
  }

  initialized = true;
  return true;
}

void Renderer::render(const Universe &universe, const Camera &camera, int windowWidth, int windowHeight)
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

  // Render satellites
  renderSatellites(universe.getSatellites());

  // Render ground stations
  renderGroundStations(universe.getGroundStations());

  // Set up line shader for line-based rendering
  lineShader->use();
  lineShader->setMat4("view", view);
  lineShader->setMat4("projection", projection);

  // Render power beams (requires selected satellite to be passed in)
  // Note: This is a limitation - we'll handle it via a separate render call
  // renderPowerBeams(universe.getGroundStations(), selectedSatellite);

  // Render orbit paths
  renderOrbitPaths(universe.getSatellites());

  // Render footprints (requires selected satellite)
  // renderFootprints(universe.getSatellites(), selectedSatellite);

  // Render attitude vectors
  renderAttitudeVectors(universe.getSatellites());

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
  sphereShader->setVec3("objectColor", glm::vec3(0.6f, 0.6f, 0.6f)); // Slightly dimmed

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

  // Apply rotation for Earth
  if (isEarth)
  {
    // Apply base rotation to align texture with coordinate system
    model = glm::rotate(model, glm::radians(90.0f), body->getRotationAxis());

    // Apply time-based rotation for Earth's spin
    model = glm::rotate(model, body->getRotation(), body->getRotationAxis());
  }

  model = glm::scale(model, glm::vec3(body->getRadius()));
  sphereShader->setMat4("model", model);
  sphereShader->setVec3("objectColor", body->getColor());

  // Set lighting properties based on body type
  if (isSun)
  {
    // Sun is self-illuminated and very bright
    glm::vec3 sunColor = body->getColor() * 3.0f; // Extra bright
    sphereShader->setVec3("objectColor", sunColor);
    sphereShader->setBool("useTexture", false);
    sphereShader->setBool("isEmissive", true);
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

void Renderer::renderSatellites(const std::vector<std::shared_ptr<Satellite>> &satellites)
{
  sphereShader->use();
  sphereShader->setBool("useTexture", false);
  sphereShader->setFloat("ambientStrength", 0.15f); // Moderate ambient for satellites
  sphereShader->setBool("isEmissive", false);       // Satellites are not self-illuminated

  for (const auto &satellite : satellites)
  {
    glm::vec3 satPos = glm::vec3(satellite->getPosition());
    glm::vec3 brightColor = satellite->getColor() * 1.5f; // Make satellites brighter
    sphereShader->setVec3("objectColor", brightColor);

    // Get satellite attitude (quaternion -> rotation matrix)
    glm::quat attitudeQuat = glm::quat(satellite->getQuaternion()); // Convert dquat to quat
    glm::mat4 attitudeMatrix = glm::mat4_cast(attitudeQuat);

    // Check if this is a Starlink satellite and we have the mesh loaded
    bool isStarlink = satellite->getName().find("Starlink") != std::string::npos;

    if (isStarlink && starlinkMeshLoaded)
    {
      // Render Starlink 3D model
      glm::mat4 meshModel = glm::mat4(1.0f);
      meshModel = glm::translate(meshModel, satPos);
      meshModel = meshModel * attitudeMatrix;               // Apply attitude rotation
      meshModel = glm::scale(meshModel, glm::vec3(0.5e5f)); // Scale to appropriate size (~100km)
      sphereShader->setMat4("model", meshModel);
      starlinkMesh.draw(*sphereShader);
    }
    else
    {
      // Fall back to simple geometry (cube + solar panels)

      // Render central body (elongated box)
      // Apply: Translation -> Rotation (attitude) -> Scale
      glm::mat4 bodyModel = glm::mat4(1.0f);
      bodyModel = glm::translate(bodyModel, satPos);
      bodyModel = bodyModel * attitudeMatrix;                               // Apply attitude rotation
      bodyModel = glm::scale(bodyModel, glm::vec3(8.0e4f, 1.2e5f, 8.0e4f)); // 80km x 120km x 80km
      sphereShader->setMat4("model", bodyModel);
      cubeMesh.draw();

      // Render left solar panel (thin flat box)
      // Solar panels extend along X-axis in body frame
      glm::mat4 leftPanelModel = glm::mat4(1.0f);
      leftPanelModel = glm::translate(leftPanelModel, satPos);
      leftPanelModel = leftPanelModel * attitudeMatrix;                                // Apply attitude rotation
      leftPanelModel = glm::translate(leftPanelModel, glm::vec3(-1.5e5f, 0.0f, 0.0f)); // Offset to the left (in body frame)
      leftPanelModel = glm::scale(leftPanelModel, glm::vec3(2.0e5f, 1.8e5f, 0.2e5f));  // 200km x 180km x 20km (thin)
      sphereShader->setMat4("model", leftPanelModel);

      // Make solar panels slightly darker (blueish tint for solar cells)
      glm::vec3 panelColor = glm::vec3(0.2f, 0.3f, 0.5f);
      sphereShader->setVec3("objectColor", panelColor);
      cubeMesh.draw();

      // Render right solar panel
      glm::mat4 rightPanelModel = glm::mat4(1.0f);
      rightPanelModel = glm::translate(rightPanelModel, satPos);
      rightPanelModel = rightPanelModel * attitudeMatrix;                               // Apply attitude rotation
      rightPanelModel = glm::translate(rightPanelModel, glm::vec3(1.5e5f, 0.0f, 0.0f)); // Offset to the right (in body frame)
      rightPanelModel = glm::scale(rightPanelModel, glm::vec3(2.0e5f, 1.8e5f, 0.2e5f)); // 200km x 180km x 20km (thin)
      sphereShader->setMat4("model", rightPanelModel);
      sphereShader->setVec3("objectColor", panelColor);
      cubeMesh.draw();
    }
  }
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

void Renderer::renderPowerBeams(const std::vector<std::shared_ptr<GroundStation>> &groundStations, const Satellite *selectedSatellite)
{
  if (!selectedSatellite)
    return;

  glLineWidth(2.0f);
  lineShader->use();

  for (const auto &groundStation : groundStations)
  {
    // Draw beams to all visible satellites
    for (const auto &satellite : groundStation->getVisibleSatellites())
    {
      if (satellite.get() != selectedSatellite)
      {
        continue;
      }

      // Draw power beam from ground station to satellite
      std::vector<glm::vec3> beamVertices;
      beamVertices.push_back(glm::vec3(groundStation->getPosition()));
      beamVertices.push_back(glm::vec3(satellite->getPosition()));

      lineRenderer.setVertices(beamVertices);

      // Use bright yellow/orange color for energy beam
      glm::vec3 beamColor = glm::vec3(1.0f, 0.8f, 0.1f); // Bright yellow-orange
      lineShader->setVec3("lineColor", beamColor);
      lineRenderer.draw();
    }
  }
}

void Renderer::renderOrbitPaths(const std::vector<std::shared_ptr<Satellite>> &satellites)
{
  glLineWidth(5.0f);
  lineShader->use();

  for (const auto &satellite : satellites)
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
      lineShader->setVec3("lineColor", orbitColor);
      lineRenderer.draw();
    }
  }
  glLineWidth(2.0f); // Reset to normal line width
}

void Renderer::renderFootprints(const std::vector<std::shared_ptr<Satellite>> &satellites, const Satellite *selectedSatellite)
{
  if (!selectedSatellite)
    return;

  lineShader->use();

  for (const auto &satellite : satellites)
  {
    if (satellite.get() != selectedSatellite)
    {
      continue;
    }

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
      lineShader->setVec3("lineColor", footprintColor);
      lineRenderer.draw();
    }
  }
}

void Renderer::renderAttitudeVectors(const std::vector<std::shared_ptr<Satellite>> &satellites)
{
  glLineWidth(3.0f); // Thicker line for attitude arrow
  lineShader->use();

  for (const auto &satellite : satellites)
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
    lineShader->setVec3("lineColor", attitudeColor);
    lineRenderer.draw();
  }
  glLineWidth(2.0f); // Reset to normal line width
}

void Renderer::renderCoordinateAxis(const Camera &camera, int windowWidth, int windowHeight)
{
  // Save current viewport
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  // Set viewport for axis display in bottom left corner
  int axisSize = 200; // Size of the axis display area (200x200 pixels)
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
  glLineWidth(3.0f);

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
