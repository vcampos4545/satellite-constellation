#ifndef SPHERE_H
#define SPHERE_H

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>

class Sphere
{
public:
  Sphere(float radius, unsigned int rings, unsigned int sectors);
  ~Sphere();

  void draw() const;

  // Prevent copying
  Sphere(const Sphere &) = delete;
  Sphere &operator=(const Sphere &) = delete;

private:
  void generateMesh(float radius, unsigned int rings, unsigned int sectors);

  GLuint VAO, VBO, EBO;
  unsigned int indexCount;
};

#endif // SPHERE_H
