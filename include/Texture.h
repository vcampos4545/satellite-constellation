#ifndef TEXTURE_H
#define TEXTURE_H

#include <GL/glew.h>
#include <string>

class Texture
{
public:
  Texture();
  ~Texture();

  bool load(const std::string &path);
  void bind(unsigned int unit = 0) const;
  void unbind() const;

  GLuint getID() const { return textureID; }
  bool isLoaded() const { return loaded; }

  // Prevent copying
  Texture(const Texture &) = delete;
  Texture &operator=(const Texture &) = delete;

private:
  GLuint textureID;
  bool loaded;
  int width;
  int height;
  int channels;
};

#endif // TEXTURE_H
