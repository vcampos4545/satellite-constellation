#ifndef LINE_RENDERER_H
#define LINE_RENDERER_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>

class LineRenderer
{
public:
  LineRenderer();
  ~LineRenderer();

  // Update the line vertices
  void setVertices(const std::vector<glm::vec3> &vertices);

  // Draw the line
  void draw() const;

  // Prevent copying
  LineRenderer(const LineRenderer &) = delete;
  LineRenderer &operator=(const LineRenderer &) = delete;

private:
  GLuint VAO, VBO;
  unsigned int vertexCount;
};

#endif // LINE_RENDERER_H
