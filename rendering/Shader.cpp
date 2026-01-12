#include "Shader.h"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <glm/gtc/type_ptr.hpp>

Shader::Shader(const char *vertexPath, const char *fragmentPath)
{
  // Load shader source code
  std::string vertexCode = loadShaderSource(vertexPath);
  std::string fragmentCode = loadShaderSource(fragmentPath);

  const char *vShaderCode = vertexCode.c_str();
  const char *fShaderCode = fragmentCode.c_str();

  // Compile vertex shader
  GLuint vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vShaderCode, NULL);
  glCompileShader(vertex);
  checkCompileErrors(vertex, "VERTEX");

  // Compile fragment shader
  GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fShaderCode, NULL);
  glCompileShader(fragment);
  checkCompileErrors(fragment, "FRAGMENT");

  // Link shader program
  ID = glCreateProgram();
  glAttachShader(ID, vertex);
  glAttachShader(ID, fragment);
  glLinkProgram(ID);
  checkCompileErrors(ID, "PROGRAM");

  // Delete shaders (they're linked into the program now)
  glDeleteShader(vertex);
  glDeleteShader(fragment);
}

Shader::~Shader()
{
  glDeleteProgram(ID);
}

void Shader::use() const
{
  glUseProgram(ID);
}

void Shader::setBool(const std::string &name, bool value) const
{
  glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}

void Shader::setInt(const std::string &name, int value) const
{
  glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setFloat(const std::string &name, float value) const
{
  glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setVec3(const std::string &name, const glm::vec3 &value) const
{
  glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(value));
}

void Shader::setVec3(const std::string &name, float x, float y, float z) const
{
  glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
}

void Shader::setMat4(const std::string &name, const glm::mat4 &mat) const
{
  glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
}

std::string Shader::loadShaderSource(const char *path)
{
  std::string code;
  std::ifstream shaderFile;

  shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  try
  {
    shaderFile.open(path);
    std::stringstream shaderStream;
    shaderStream << shaderFile.rdbuf();
    shaderFile.close();
    code = shaderStream.str();
  }
  catch (std::ifstream::failure &e)
  {
    printf("\033[31mERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: %s\033[0m\n", path);
    printf("\033[31mError: %s\033[0m\n", e.what());
  }

  return code;
}

void Shader::checkCompileErrors(GLuint shader, const std::string &type)
{
  GLint success;
  GLchar infoLog[1024];

  if (type != "PROGRAM")
  {
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
      glGetShaderInfoLog(shader, 1024, NULL, infoLog);
      printf("\033[31mERROR::SHADER_COMPILATION_ERROR of type: %s\n%s\033[0m\n",
             type.c_str(), infoLog);
    }
  }
  else
  {
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success)
    {
      glGetProgramInfoLog(shader, 1024, NULL, infoLog);
      printf("\033[31mERROR::PROGRAM_LINKING_ERROR of type: %s\n%s\033[0m\n",
             type.c_str(), infoLog);
    }
  }
}
