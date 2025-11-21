#ifndef CUBE_H
#define CUBE_H

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>

class Cube
{
public:
  Cube();
  ~Cube();

  void draw() const;

  // Prevent copying
  Cube(const Cube &) = delete;
  Cube &operator=(const Cube &) = delete;

private:
  void generateMesh();

  GLuint VAO, VBO, EBO;
  unsigned int indexCount;
};

#endif // CUBE_H
