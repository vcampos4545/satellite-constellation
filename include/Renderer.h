#ifndef RENDERER_H
#define RENDERER_H

#include <GL/glew.h>
#include <memory>
#include <string>
#include "Universe.h"
#include "Camera.h"
#include "Shader.h"
#include "Sphere.h"
#include "Cube.h"
#include "LineRenderer.h"
#include "Texture.h"
#include "OBJMesh.h"

/**
 * Renderer class - Separates rendering logic from simulation
 *
 * Responsibilities:
 * - Manages all OpenGL resources (shaders, meshes, textures)
 * - Renders the complete scene (celestial bodies, satellites, orbits, etc.)
 * - Provides clean interface: renderer.render(universe, camera)
 *
 * Benefits:
 * - Physics classes stay pure (no OpenGL dependencies)
 * - Easy to run simulation headless for Monte Carlo studies
 * - Can swap rendering backends without changing physics
 * - Cleaner, more maintainable code
 */
class Renderer
{
public:
  Renderer();
  ~Renderer();

  // Initialize renderer (load shaders, create meshes)
  bool initialize();

  // Main render function - renders entire scene
  void render(const Universe &universe, const Camera &camera, int windowWidth, int windowHeight);

  // Render individual components
  void renderStarBackground(const Camera &camera);
  void renderCelestialBody(const std::shared_ptr<CelestialBody> &body, bool isEarth, bool isMoon, bool isSun);
  void renderSatellites(const std::vector<std::shared_ptr<Satellite>> &satellites);
  void renderGroundStations(const std::vector<std::shared_ptr<GroundStation>> &groundStations);
  void renderPowerBeams(const std::vector<std::shared_ptr<GroundStation>> &groundStations, const Satellite *selectedSatellite);
  void renderOrbitPaths(const std::vector<std::shared_ptr<Satellite>> &satellites);
  void renderFootprints(const std::vector<std::shared_ptr<Satellite>> &satellites, const Satellite *selectedSatellite);
  void renderAttitudeVectors(const std::vector<std::shared_ptr<Satellite>> &satellites);
  void renderCoordinateAxis(const Camera &camera, int windowWidth, int windowHeight);

  // Getters for checking initialization status
  bool hasEarthTexture() const { return earthTextureLoaded; }
  bool hasMoonTexture() const { return moonTextureLoaded; }
  bool hasStarsTexture() const { return starsTextureLoaded; }

  // Prevent copying
  Renderer(const Renderer &) = delete;
  Renderer &operator=(const Renderer &) = delete;

private:
  // Shaders (using unique_ptr since Shader has no default constructor)
  std::unique_ptr<Shader> sphereShader;
  std::unique_ptr<Shader> lineShader;

  // Meshes
  Sphere sphereMesh;
  Cube cubeMesh;
  LineRenderer lineRenderer;
  OBJMesh starlinkMesh;

  // Textures
  Texture earthTexture;
  Texture moonTexture;
  Texture starsTexture;

  // Texture loaded flags
  bool earthTextureLoaded;
  bool moonTextureLoaded;
  bool starsTextureLoaded;

  // Mesh loaded flags
  bool starlinkMeshLoaded;

  // Initialization state
  bool initialized;
};

#endif // RENDERER_H
