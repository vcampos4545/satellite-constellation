#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <string>
#include <glm/glm.hpp>

class Shader
{
public:
  Shader(const char *vertexPath, const char *fragmentPath);
  ~Shader();

  void use() const;

  // Utility functions
  void setBool(const std::string &name, bool value) const;
  void setInt(const std::string &name, int value) const;
  void setFloat(const std::string &name, float value) const;
  void setVec3(const std::string &name, const glm::vec3 &value) const;
  void setVec3(const std::string &name, float x, float y, float z) const;
  void setMat4(const std::string &name, const glm::mat4 &mat) const;

  GLuint getID() const { return ID; }

private:
  GLuint ID;

  std::string loadShaderSource(const char *path);
  void checkCompileErrors(GLuint shader, const std::string &type);
};

#endif // SHADER_H
