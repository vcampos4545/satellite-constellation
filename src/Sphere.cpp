#include "Sphere.h"
#include <cmath>

Sphere::Sphere(float radius, unsigned int rings, unsigned int sectors)
    : VAO(0), VBO(0), EBO(0), indexCount(0)
{
  generateMesh(radius, rings, sectors);
}

Sphere::~Sphere()
{
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &EBO);
}

void Sphere::generateMesh(float radius, unsigned int rings, unsigned int sectors)
{
  std::vector<float> vertices;
  std::vector<unsigned int> indices;

  const float PI = 3.14159265359f;
  const float R = 1.0f / static_cast<float>(rings - 1);
  const float S = 1.0f / static_cast<float>(sectors - 1);

  // Generate vertices
  // Z-up coordinate system: Z is vertical, X is vernal equinox (0° longitude)
  for (unsigned int r = 0; r < rings; ++r)
  {
    for (unsigned int s = 0; s < sectors; ++s)
    {
      float z = sin(-PI / 2.0f + PI * r * R);
      float x = -cos(2 * PI * s * S) * sin(PI * r * R); // Negate X to fix winding order
      float y = sin(2 * PI * s * S) * sin(PI * r * R);

      // Position
      vertices.push_back(x * radius);
      vertices.push_back(y * radius);
      vertices.push_back(z * radius);

      // Normal (normalized position for a sphere)
      vertices.push_back(x);
      vertices.push_back(y);
      vertices.push_back(z);

      // Texture coordinates (UV) - flip U to fix backwards texture
      vertices.push_back(1.0f - s * S);
      vertices.push_back(r * R);
    }
  }

  // Generate indices
  for (unsigned int r = 0; r < rings - 1; ++r)
  {
    for (unsigned int s = 0; s < sectors - 1; ++s)
    {
      unsigned int curRow = r * sectors;
      unsigned int nextRow = (r + 1) * sectors;

      indices.push_back(curRow + s);
      indices.push_back(nextRow + s);
      indices.push_back(nextRow + s + 1);

      indices.push_back(curRow + s);
      indices.push_back(nextRow + s + 1);
      indices.push_back(curRow + s + 1);
    }
  }

  indexCount = indices.size();

  // Create buffers
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  // Normal attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // Texture coordinate attribute
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);

  glBindVertexArray(0);
}

void Sphere::draw() const
{
  glBindVertexArray(VAO);
  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}
