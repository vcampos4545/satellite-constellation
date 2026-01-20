#include "Texture.h"
#include <cstdio>

#define STB_IMAGE_IMPLEMENTATION
#include "../external/stb_image.h"

Texture::Texture()
    : textureID(0), loaded(false), width(0), height(0), channels(0)
{
}

Texture::~Texture()
{
  if (textureID != 0)
  {
    glDeleteTextures(1, &textureID);
  }
}

bool Texture::load(const std::string &path)
{
  // Load image data
  stbi_set_flip_vertically_on_load(true);
  unsigned char *data = stbi_load(path.c_str(), &width, &height, &channels, 0);

  if (!data)
  {
    printf("\033[31mFailed to load texture: %s\033[0m\n", path.c_str());
    printf("\033[31mSTB Image Error: %s\033[0m\n", stbi_failure_reason());
    return false;
  }

  // Determine format
  GLenum format;
  if (channels == 1)
    format = GL_RED;
  else if (channels == 3)
    format = GL_RGB;
  else if (channels == 4)
    format = GL_RGBA;
  else
  {
    printf("\033[31mUnsupported number of channels: %d\033[0m\n", channels);
    stbi_image_free(data);
    return false;
  }

  // Generate texture
  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_2D, textureID);

  // Set texture parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // Upload texture data
  glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
  glGenerateMipmap(GL_TEXTURE_2D);

  // Free image data
  stbi_image_free(data);

  loaded = true;
  printf("\033[32mLoaded texture: %s (%dx%d, %d channels)\033[0m\n",
         path.c_str(), width, height, channels);

  return true;
}

void Texture::bind(unsigned int unit) const
{
  glActiveTexture(GL_TEXTURE0 + unit);
  glBindTexture(GL_TEXTURE_2D, textureID);
}

void Texture::unbind() const
{
  glBindTexture(GL_TEXTURE_2D, 0);
}
