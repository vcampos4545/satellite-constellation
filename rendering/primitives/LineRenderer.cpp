#include "LineRenderer.h"

LineRenderer::LineRenderer()
    : VAO(0), VBO(0), vertexCount(0)
{
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);

  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
}

LineRenderer::~LineRenderer()
{
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
}

void LineRenderer::setVertices(const std::vector<glm::vec3> &vertices)
{
  vertexCount = vertices.size();

  if (vertexCount == 0)
    return;

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void LineRenderer::draw() const
{
  if (vertexCount < 2)
    return;

  glBindVertexArray(VAO);
  glDrawArrays(GL_LINE_STRIP, 0, vertexCount);
  glBindVertexArray(0);
}
