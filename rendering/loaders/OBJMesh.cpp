#include "OBJMesh.h"
#include "Shader.h"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <set>

OBJMesh::OBJMesh()
    : VAO(0), VBO(0), EBO(0)
{
}

OBJMesh::~OBJMesh()
{
  if (VAO != 0)
  {
    glDeleteVertexArrays(1, &VAO);
  }
  if (VBO != 0)
  {
    glDeleteBuffers(1, &VBO);
  }
  if (EBO != 0)
  {
    glDeleteBuffers(1, &EBO);
  }
}

bool OBJMesh::load(const std::string &filepath)
{
  // Open the OBJ file
  std::ifstream file(filepath);
  if (!file.is_open())
  {
    printf("\033[31mFailed to open OBJ file: %s\033[0m\n", filepath.c_str());
    return false;
  }

  // Temporary storage for vertex attributes
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec2> texCoords;

  // Temporary storage for face indices
  struct FaceVertex
  {
    int posIndex;
    int texIndex;
    int normIndex;
  };
  std::vector<FaceVertex> faceVertices;

  std::string currentMaterial;
  std::string currentGroup;
  int currentGroupStartIndex = 0;

  std::string line;
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string prefix;
    iss >> prefix;

    // Vertex position
    if (prefix == "v")
    {
      glm::vec3 pos;
      iss >> pos.x >> pos.y >> pos.z;
      positions.push_back(pos);
    }
    // Vertex normal
    else if (prefix == "vn")
    {
      glm::vec3 norm;
      iss >> norm.x >> norm.y >> norm.z;
      normals.push_back(norm);
    }
    // Vertex texture coordinate
    else if (prefix == "vt")
    {
      glm::vec2 tex;
      iss >> tex.x >> tex.y;
      texCoords.push_back(tex);
    }
    // Face
    else if (prefix == "f")
    {
      std::string vertexStr;
      std::vector<FaceVertex> faceVerts;

      while (iss >> vertexStr)
      {
        FaceVertex fv = {-1, -1, -1};

        // Parse vertex data in format: v/vt/vn or v//vn or v/vt or v
        std::replace(vertexStr.begin(), vertexStr.end(), '/', ' ');
        std::istringstream viss(vertexStr);

        viss >> fv.posIndex;
        if (!viss.eof())
        {
          viss >> fv.texIndex;
          if (!viss.eof())
          {
            viss >> fv.normIndex;
          }
        }

        faceVerts.push_back(fv);
      }

      // Triangulate face (assumes convex polygons)
      // Fan triangulation: vertex 0, 1, 2, then 0, 2, 3, etc.
      for (size_t i = 1; i + 1 < faceVerts.size(); ++i)
      {
        faceVertices.push_back(faceVerts[0]);
        faceVertices.push_back(faceVerts[i]);
        faceVertices.push_back(faceVerts[i + 1]);
      }
    }
    // Material library
    else if (prefix == "mtllib")
    {
      std::string mtlFile;
      iss >> mtlFile;

      // Extract directory from filepath
      size_t lastSlash = filepath.find_last_of("/\\");
      std::string directory = (lastSlash != std::string::npos) ? filepath.substr(0, lastSlash + 1) : "";

      loadMTL(directory + mtlFile);
    }
    // Use material
    else if (prefix == "usemtl")
    {
      // Save current group if it has faces
      if (currentGroupStartIndex < (int)faceVertices.size())
      {
        MeshGroup group;
        group.name = currentGroup.empty() ? "default" : currentGroup;
        group.materialName = currentMaterial;
        group.startIndex = currentGroupStartIndex; // Index into the indices array
        group.indexCount = (int)faceVertices.size() - currentGroupStartIndex;
        groups.push_back(group);
      }

      iss >> currentMaterial;
      currentGroupStartIndex = (int)faceVertices.size();
    }
    // Group
    else if (prefix == "g" || prefix == "o")
    {
      iss >> currentGroup;
    }
  }

  file.close();

  // Save final group
  if (currentGroupStartIndex < (int)faceVertices.size())
  {
    MeshGroup group;
    group.name = currentGroup.empty() ? "default" : currentGroup;
    group.materialName = currentMaterial;
    group.startIndex = currentGroupStartIndex;
    group.indexCount = (int)faceVertices.size() - currentGroupStartIndex;
    groups.push_back(group);
  }

  // If no groups were created, create a default group
  if (groups.empty() && !faceVertices.empty())
  {
    MeshGroup group;
    group.name = "default";
    group.materialName = currentMaterial;
    group.startIndex = 0;
    group.indexCount = (int)faceVertices.size();
    groups.push_back(group);
  }

  // Build vertex and index buffers
  std::vector<unsigned int> indices;
  std::map<std::string, unsigned int> vertexCache;

  for (const auto &fv : faceVertices)
  {
    // Create unique vertex key
    std::string key = std::to_string(fv.posIndex) + "/" +
                      std::to_string(fv.texIndex) + "/" +
                      std::to_string(fv.normIndex);

    // Check if this vertex combination already exists
    auto it = vertexCache.find(key);
    if (it != vertexCache.end())
    {
      indices.push_back(it->second);
    }
    else
    {
      // Add new vertex to buffer
      unsigned int newIndex = (unsigned int)(vertices.size() / 8);

      // Position (OBJ indices are 1-based)
      if (fv.posIndex > 0 && fv.posIndex <= (int)positions.size())
      {
        glm::vec3 pos = positions[fv.posIndex - 1];
        vertices.push_back(pos.x);
        vertices.push_back(pos.y);
        vertices.push_back(pos.z);
      }
      else
      {
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
      }

      // Normal
      if (fv.normIndex > 0 && fv.normIndex <= (int)normals.size())
      {
        glm::vec3 norm = normals[fv.normIndex - 1];
        vertices.push_back(norm.x);
        vertices.push_back(norm.y);
        vertices.push_back(norm.z);
      }
      else
      {
        // Default normal (up)
        vertices.push_back(0.0f);
        vertices.push_back(1.0f);
        vertices.push_back(0.0f);
      }

      // Texture coordinates
      if (fv.texIndex > 0 && fv.texIndex <= (int)texCoords.size())
      {
        glm::vec2 tex = texCoords[fv.texIndex - 1];
        vertices.push_back(tex.x);
        vertices.push_back(tex.y);
      }
      else
      {
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
      }

      indices.push_back(newIndex);
      vertexCache[key] = newIndex;
    }
  }

  // Create OpenGL buffers
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);

  // Upload vertex data
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

  // Upload index data
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

  // Position attribute (location 0)
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  // Normal attribute (location 1)
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // Texture coordinate attribute (location 2)
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);

  glBindVertexArray(0);

  // Validate the mesh data
  if (vertices.empty() || indices.empty())
  {
    printf("\033[31mError: OBJ mesh has no geometry data\033[0m\n");
    return false;
  }

  printf("\033[32mLoaded OBJ mesh: %s\033[0m\n", filepath.c_str());
  printf("  Vertices: %lu\n", vertices.size() / 8);
  printf("  Indices: %lu\n", indices.size());
  printf("  Groups: %lu\n", groups.size());
  printf("  Materials: %lu\n", materials.size());
  printf("  VAO: %u, VBO: %u, EBO: %u\n", VAO, VBO, EBO);

  return true;
}

bool OBJMesh::loadMTL(const std::string &filepath)
{
  // printf("Attempting to load MTL file: %s\n", filepath.c_str());
  std::ifstream file(filepath);
  if (!file.is_open())
  {
    // printf("\033[31mFailed to open MTL file: %s\033[0m\n", filepath.c_str());
    return false;
  }
  // printf("\033[32mMTL file opened successfully: %s\033[0m\n", filepath.c_str());

  Material currentMaterial;
  std::string currentMaterialName;

  std::string line;
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string prefix;
    iss >> prefix;

    if (prefix == "newmtl")
    {
      // Save previous material
      if (!currentMaterialName.empty())
      {
        materials[currentMaterialName] = currentMaterial;
      }

      // Start new material
      iss >> currentMaterialName;
      currentMaterial = Material(); // Reset to defaults
      currentMaterial.name = currentMaterialName;
    }
    else if (prefix == "Ka")
    {
      iss >> currentMaterial.ambient.r >> currentMaterial.ambient.g >> currentMaterial.ambient.b;
    }
    else if (prefix == "Kd")
    {
      iss >> currentMaterial.diffuse.r >> currentMaterial.diffuse.g >> currentMaterial.diffuse.b;
    }
    else if (prefix == "Ks")
    {
      iss >> currentMaterial.specular.r >> currentMaterial.specular.g >> currentMaterial.specular.b;
    }
    else if (prefix == "Ns")
    {
      iss >> currentMaterial.shininess;
    }
    else if (prefix == "map_Kd")
    {
      iss >> currentMaterial.diffuseMap;
    }
  }

  // Save final material
  if (!currentMaterialName.empty())
  {
    materials[currentMaterialName] = currentMaterial;
  }

  file.close();

  // printf("\033[32mLoaded %lu materials from MTL file\033[0m\n", materials.size());
  // for (const auto &matPair : materials)
  // {
  //   printf("  Material: %s - Diffuse RGB(%.5f, %.5f, %.5f)\n",
  //          matPair.first.c_str(),
  //          matPair.second.diffuse.r,
  //          matPair.second.diffuse.g,
  //          matPair.second.diffuse.b);
  // }

  // Load textures for all materials
  // Extract base directory from MTL filepath
  size_t lastSlash = filepath.find_last_of("/\\");
  std::string baseDir = (lastSlash != std::string::npos) ? filepath.substr(0, lastSlash + 1) : "";

  for (auto &matPair : materials)
  {
    Material &mat = matPair.second;
    if (!mat.diffuseMap.empty())
    {
      // Create texture and try to load it
      auto texture = std::make_shared<Texture>();
      std::string texturePath = baseDir + mat.diffuseMap;

      if (texture->load(texturePath))
      {
        mat.diffuseTexture = texture;
        // printf("  \033[32mLoaded texture: %s\033[0m\n", texturePath.c_str());
      }
      else
      {
        // printf("  \033[31mFailed to load texture: %s\033[0m\n", texturePath.c_str());
      }
    }
  }

  return true;
}

void OBJMesh::draw(Shader &shader) const
{
  // Safety check: don't try to draw if mesh wasn't loaded successfully
  if (VAO == 0 || groups.empty())
  {
    return;
  }

  // Debug output (only on first draw call for this mesh)
  // We track this per-instance to avoid spamming the console every frame
  static std::set<const OBJMesh *> debugPrintedMeshes;
  if (debugPrintedMeshes.find(this) == debugPrintedMeshes.end())
  {
    printf("Drawing OBJ mesh with %lu groups and %lu materials\n", groups.size(), materials.size());
    for (const auto &group : groups)
    {
      printf("  Group '%s' uses material '%s'\n", group.name.c_str(), group.materialName.c_str());
    }
    debugPrintedMeshes.insert(this);
  }

  glBindVertexArray(VAO);

  // Draw each group with its material
  for (const auto &group : groups)
  {
    // Skip empty groups
    if (group.indexCount == 0)
    {
      continue;
    }

    // Apply material properties
    if (!group.materialName.empty())
    {
      auto it = materials.find(group.materialName);
      if (it != materials.end())
      {
        const Material &mat = it->second;

        // If material has a texture, use it
        if (mat.diffuseTexture && mat.diffuseTexture->isLoaded())
        {
          shader.setBool("useTexture", true);
          mat.diffuseTexture->bind(0);
          shader.setInt("textureSampler", 0);
        }
        else
        {
          // No texture, use material diffuse color
          shader.setBool("useTexture", false);
          shader.setVec3("objectColor", mat.diffuse);
        }
      }
      else
      {
        // Material not found - use default gray color
        shader.setBool("useTexture", false);
        shader.setVec3("objectColor", glm::vec3(0.5f, 0.5f, 0.5f));
        printf("\033[33mWarning: Material '%s' not found for group '%s'\033[0m\n",
               group.materialName.c_str(), group.name.c_str());
      }
    }
    else
    {
      // No material assigned - use default white color
      shader.setBool("useTexture", false);
      shader.setVec3("objectColor", glm::vec3(1.0f, 1.0f, 1.0f));
    }

    glDrawElements(GL_TRIANGLES, group.indexCount, GL_UNSIGNED_INT, (void *)(group.startIndex * sizeof(unsigned int)));
  }

  glBindVertexArray(0);
}

size_t OBJMesh::getTriangleCount() const
{
  size_t totalIndices = 0;
  for (const auto &group : groups)
  {
    totalIndices += group.indexCount;
  }
  return totalIndices / 3;
}
