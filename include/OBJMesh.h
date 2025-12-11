#ifndef OBJMESH_H
#define OBJMESH_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <map>

/**
 * Material properties from MTL file
 */
struct Material
{
  std::string name;
  glm::vec3 ambient;      // Ka - Ambient color
  glm::vec3 diffuse;      // Kd - Diffuse color
  glm::vec3 specular;     // Ks - Specular color
  float shininess;        // Ns - Shininess exponent
  std::string diffuseMap; // map_Kd - Diffuse texture filename (optional)

  Material()
      : name("default"),
        ambient(0.2f, 0.2f, 0.2f),
        diffuse(0.8f, 0.8f, 0.8f),
        specular(0.0f, 0.0f, 0.0f),
        shininess(0.0f),
        diffuseMap("")
  {
  }
};

/**
 * Mesh group with single material
 */
struct MeshGroup
{
  std::string name;         // Group name
  std::string materialName; // Material name
  int startIndex;           // Starting index in the EBO
  int indexCount;           // Number of indices

  MeshGroup() : name(""), materialName(""), startIndex(0), indexCount(0) {}
};

/**
 * OBJMesh - Loads and renders Wavefront OBJ files
 *
 * Features:
 * - Loads vertex positions, normals, and texture coordinates
 * - Supports MTL material files
 * - Handles multiple materials per mesh
 * - Efficient indexed rendering
 * - Compatible with existing sphere shader
 *
 * Usage:
 *   OBJMesh satelliteModel;
 *   satelliteModel.load("models/starlink.obj");
 *
 *   // In render loop:
 *   shader.use();
 *   shader.setMat4("model", modelMatrix);
 *   satelliteModel.draw(shader);
 */
class OBJMesh
{
public:
  OBJMesh();
  ~OBJMesh();

  // Load OBJ file (automatically loads MTL if referenced)
  bool load(const std::string &filepath);

  // Render the mesh (sets material colors, draws all groups)
  void draw(class Shader &shader) const;

  // Get mesh info
  size_t getVertexCount() const { return vertices.size() / 8; }
  size_t getTriangleCount() const;
  size_t getMaterialCount() const { return materials.size(); }

  // Prevent copying (OpenGL resources)
  OBJMesh(const OBJMesh &) = delete;
  OBJMesh &operator=(const OBJMesh &) = delete;

private:
  // OpenGL buffers
  GLuint VAO;
  GLuint VBO;
  GLuint EBO;

  // Vertex data: [x, y, z, nx, ny, nz, u, v] per vertex
  std::vector<float> vertices;

  // Mesh groups (one per material)
  std::vector<MeshGroup> groups;

  // Materials loaded from MTL file
  std::map<std::string, Material> materials;

  // Base directory for resolving relative paths
  std::string baseDir;

  // Loading functions
  bool loadMTL(const std::string &filepath);
  bool parseOBJ(const std::string &filepath);
  void setupMesh();

  // Helper functions
  std::string getDirectory(const std::string &filepath);
  std::string trim(const std::string &str);
};

#endif // OBJMESH_H
