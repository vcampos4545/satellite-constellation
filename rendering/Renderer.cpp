#include "Renderer.h"
#include <iostream>
#include <filesystem>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Constants.h"
#include "OBJMesh.h"
#include "VisualizationState.h"

namespace fs = std::filesystem;

// Helper
static void buildOrthonormalBasis(
    const glm::vec3 &n,
    glm::vec3 &b1,
    glm::vec3 &b2)
{
  if (fabs(n.x) > fabs(n.z))
    b1 = glm::normalize(glm::vec3(-n.y, n.x, 0.0f));
  else
    b1 = glm::normalize(glm::vec3(0.0f, -n.z, n.y));

  b2 = glm::normalize(glm::cross(n, b1));
}

Renderer::Renderer()
    : sphereMesh(1.0f, 30, 30), // Unit sphere with 30x30 tessellation
      earthTextureLoaded(false),
      moonTextureLoaded(false),
      sunTextureLoaded(false),
      starsTextureLoaded(false),
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

  // Dynamically load all satellite models from models/ directory
  loadAllModels();

  initialized = true;
  return true;
}

void Renderer::render(const Universe &universe, const Camera &camera, int windowWidth, int windowHeight,
                      VisualizationState &vizState, const Spacecraft *selectedSpacecraft)
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

  // Render Moon's orbit path
  renderLine(universe.getMoon()->getOrbitPath(), glm::vec3(1.0f, 1.0f, 1.0f), 10.0f);

  // Render spacecrafts with all visualization options
  renderSpacecrafts(universe.getSpacecrafts(), vizState, selectedSpacecraft);

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
  sphereShader->setVec3("objectColor", glm::vec3(0.0f, 0.0f, 1.0f)); // Blue default

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

void Renderer::renderSpacecrafts(const std::vector<std::shared_ptr<Spacecraft>> &spacecrafts,
                                 VisualizationState &vizState,
                                 const Spacecraft *selectedSpacecraft)
{
  // Single loop through all satellites, rendering each component as needed
  for (const auto &sc : spacecrafts)
  {
    // Render satellite 3D geometry (always rendered)
    renderSpacecraftGeometry(sc);
    auto &scViz = vizState.getOrCreate(sc.get());

    // Save orbit path
    // TODO: Only add every n frames
    scViz.orbitPath.push_back(sc->getPosition());

    if (scViz.showOrbitPath)
    {
      renderLine(scViz.orbitPath, glm::vec3(1.0f, 1.0f, 1.0f), 10.0f); // White, 10px wide
    }

    if (scViz.showPredictedOrbitPath)
    {
      renderLine(scViz.predictedOrbitPath, glm::vec3(0.3f, 0.8f, 1.0f), 10.0f); // Cyan, 10px wide
    }

    if (scViz.showVelocityVector)
    {
      renderSpacecraftVelocityVector(sc);
    }

    if (scViz.showAttitudeVector)
    {
      renderSpacecraftAttitudeVector(sc);
    }

    if (scViz.showAxes)
    {
      renderSpacecraftAxes(sc);
    }
  }
}

// ========================================
// SPACECRAFT RENDER HELPER FUNCTIONS
// ========================================

// Generic line rendering helper - dvec3 version (converts to vec3)
void Renderer::renderLine(const std::vector<glm::dvec3> &vertices, const glm::vec3 &color, float lineWidth)
{
  if (vertices.size() < 2)
    return;

  // Convert dvec3 to vec3 for rendering
  std::vector<glm::vec3> convertedVertices;
  convertedVertices.reserve(vertices.size());
  for (const auto &pos : vertices)
  {
    convertedVertices.push_back(glm::vec3(pos));
  }

  // Call the vec3 version
  renderLine(convertedVertices, color, lineWidth);
}

// Generic line rendering helper - vec3 version (direct rendering)
void Renderer::renderLine(const std::vector<glm::vec3> &vertices, const glm::vec3 &color, float lineWidth)
{
  if (vertices.size() < 2)
    return;

  lineRenderer.setVertices(vertices);

  // Setup line rendering
  glLineWidth(lineWidth);
  lineShader->use();
  lineShader->setVec3("lineColor", color);
  lineRenderer.draw();
  glLineWidth(2.0f); // Reset to default
}

void Renderer::renderArrow(
    const glm::vec3 &start,
    const glm::vec3 &end,
    const glm::vec3 &color,
    float lineWidth,
    float headLength,
    float headRadius,
    int coneSegments)
{
  glm::vec3 dir = end - start;
  float length = glm::length(dir);
  if (length <= 0.0001f)
    return;

  glm::vec3 dirNorm = glm::normalize(dir);

  // Clamp head length
  headLength = glm::min(headLength, length * 0.5f);

  glm::vec3 shaftEnd = end - dirNorm * headLength;

  // ---- Draw shaft ----
  renderLine(
      std::vector<glm::vec3>{start, shaftEnd},
      color,
      lineWidth);

  // ---- Build cone basis ----
  glm::vec3 b1, b2;
  buildOrthonormalBasis(dirNorm, b1, b2);

  // ---- Draw cone edges ----
  std::vector<glm::vec3> coneLines;

  for (int i = 0; i < coneSegments; ++i)
  {
    float a0 = (float)i / coneSegments * glm::two_pi<float>();
    float a1 = (float)(i + 1) / coneSegments * glm::two_pi<float>();

    glm::vec3 p0 = shaftEnd + (cos(a0) * b1 + sin(a0) * b2) * headRadius;

    glm::vec3 p1 = shaftEnd + (cos(a1) * b1 + sin(a1) * b2) * headRadius;

    // Circle edge
    coneLines.push_back(p0);
    coneLines.push_back(p1);

    // Side edge to tip
    coneLines.push_back(p0);
    coneLines.push_back(end);
  }

  renderLine(coneLines, color, lineWidth);
}

void Renderer::renderSpacecraftGeometry(const std::shared_ptr<Spacecraft> &spacecraft)
{
  glm::vec3 scPos = glm::vec3(spacecraft->getPosition());

  // Get spacecraft attitude (quaternion -> rotation matrix)
  glm::quat attitudeQuat = glm::quat(spacecraft->getAttitude());
  glm::mat4 attitudeMatrix = glm::mat4_cast(attitudeQuat);

  // Setup shader for satellite rendering
  sphereShader->use();
  sphereShader->setBool("useTexture", false);
  sphereShader->setFloat("ambientStrength", 1.0f);
  sphereShader->setBool("isEmissive", true); // Spacecrafts are self-illuminated

  std::string modelName = spacecraft->getModelName();
  // Try to find and render the 3D model
  auto modelIt = objModels.find(modelName);
  if (!modelName.empty() && modelIt != objModels.end())
  {
    // Render 3D model with materials
    glm::mat4 meshModel = glm::mat4(1.0f);
    meshModel = glm::translate(meshModel, scPos);
    meshModel = meshModel * attitudeMatrix;
    meshModel = glm::scale(meshModel, glm::vec3(1e4f)); // ~8km scale
    sphereShader->setMat4("model", meshModel);

    modelIt->second->draw(*sphereShader);
  }
  else
  {
    // Default to satellite model
    // TODO: Make fallback when satellite model doesnt exist
    auto model = objModels.find("satellite");

    // Render 3D model with materials
    glm::mat4 meshModel = glm::mat4(1.0f);
    meshModel = glm::translate(meshModel, scPos);
    meshModel = meshModel * attitudeMatrix;
    meshModel = glm::scale(meshModel, glm::vec3(5.0f));
    sphereShader->setMat4("model", meshModel);

    model->second->draw(*sphereShader);
  }
}

void Renderer::renderSpacecraftAttitudeVector(const std::shared_ptr<Spacecraft> &spacecraft)
{
  // Get satellite position and pointing direction (Z-axis in body frame)
  glm::dvec3 scPos = spacecraft->getPosition();
  glm::dvec3 zAxis = spacecraft->getBodyZAxis();

  // Scale arrow to be visible (500 km long)
  double arrowLength = 100e3;
  glm::dvec3 arrowEnd = scPos + zAxis * arrowLength;

  renderArrow(scPos, arrowEnd, glm::vec3(1.0f, 0.0f, 1.0f), 3.0f); // Purple, 3px wide
}

void Renderer::renderSpacecraftVelocityVector(const std::shared_ptr<Spacecraft> &spacecraft)
{
  // Get satellite position and pointing direction (Z-axis in body frame)
  glm::dvec3 scPos = spacecraft->getPosition();
  glm::dvec3 velDir = glm::normalize(spacecraft->getVelocity());

  // Scale arrow to be visible (500 km long)
  double arrowLength = 100e3;
  glm::dvec3 arrowEnd = scPos + velDir * arrowLength;

  renderArrow(scPos, arrowEnd, glm::vec3(1.0f, 1.0f, 0.0f), 3.0f); // Yellow, 3px wide
}

void Renderer::renderSpacecraftAxes(const std::shared_ptr<Spacecraft> &spacecraft)
{
  glm::dvec3 scPos = spacecraft->getPosition();
  double arrowLength = 100e3;
  // X axis
  auto xAxis = spacecraft->getBodyXAxis();
  renderArrow(scPos, scPos + xAxis * arrowLength, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f);
  // Y axis
  auto yAxis = spacecraft->getBodyYAxis();
  renderArrow(scPos, scPos + yAxis * arrowLength, glm::vec3(0.0f, 1.0f, 0.0f), 10.0f);
  // Z axis
  auto zAxis = spacecraft->getBodyZAxis();
  renderArrow(scPos, scPos + zAxis * arrowLength, glm::vec3(0.0f, 0.0f, 1.0f), 10.0f);
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

// ========================================
// MODEL LOADING HELPERS
// ========================================

void Renderer::loadAllModels()
{
  const std::string modelsDir = "models";

  // Check if models directory exists
  if (!fs::exists(modelsDir) || !fs::is_directory(modelsDir))
  {
    std::cout << "Note: models/ directory not found. Using simple geometry for all satellites." << std::endl;
    return;
  }

  // Iterate through all subdirectories in models/
  for (const auto &entry : fs::directory_iterator(modelsDir))
  {
    if (!entry.is_directory())
      continue;

    std::string folderName = entry.path().filename().string();
    std::string objPath = entry.path().string() + "/" + folderName + ".obj";

    // Try to load the .obj file with the same name as the folder
    if (fs::exists(objPath))
    {
      auto mesh = std::make_shared<OBJMesh>();
      if (mesh->load(objPath))
      {
        objModels[folderName] = mesh;
        std::cout << "Loaded satellite model: " << folderName << std::endl;
      }
      else
      {
        std::cout << "Warning: Failed to load model " << objPath << std::endl;
      }
    }
    else
    {
      std::cout << "Note: Expected model file not found: " << objPath << std::endl;
    }
  }

  std::cout << "Total satellite models loaded: " << objModels.size() << std::endl;
}
