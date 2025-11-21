#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera
{
public:
  Camera(float fov, float aspectRatio, float nearPlane, float farPlane);

  // Get matrices
  glm::mat4 getViewMatrix() const;
  glm::mat4 getProjectionMatrix() const { return projectionMatrix; }

  // Camera controls (orbital around target)
  void setTarget(const glm::vec3 &target) { this->target = target; }
  void setDistance(float distance) { this->distance = distance; }
  void setAngles(float theta, float phi);

  // Adjust distance (for zooming)
  void adjustDistance(float delta);

  // Update aspect ratio (for window resize)
  void setAspectRatio(float aspectRatio);

  // Getters
  glm::vec3 getPosition() const { return position; }
  float getDistance() const { return distance; }

private:
  void updatePosition();

  // Projection parameters
  float fov;
  float aspectRatio;
  float nearPlane;
  float farPlane;
  glm::mat4 projectionMatrix;

  // Camera position (orbital)
  glm::vec3 target;    // What we're looking at
  glm::vec3 position;  // Camera position
  float distance;      // Distance from target
  float theta;         // Horizontal angle (azimuth)
  float phi;           // Vertical angle (elevation)
};

#endif // CAMERA_H
