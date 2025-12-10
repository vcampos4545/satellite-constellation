#include "Camera.h"
#include <glm/gtc/constants.hpp>

Camera::Camera(float fov, float aspectRatio, float nearPlane, float farPlane)
    : fov(fov), aspectRatio(aspectRatio), nearPlane(nearPlane), farPlane(farPlane),
      target(0.0f, 0.0f, 0.0f), distance(2e7f), theta(0.0f), phi(glm::pi<float>() / 4.0f)
{
  projectionMatrix = glm::perspective(glm::radians(fov), aspectRatio, nearPlane, farPlane);
  updatePosition();
}

void Camera::updatePosition()
{
  // Convert spherical coordinates to Cartesian
  float x = distance * cos(phi) * cos(theta);
  float y = distance * sin(phi);
  float z = distance * cos(phi) * sin(theta);

  position = target + glm::vec3(x, y, z);
}

glm::mat4 Camera::getViewMatrix() const
{
  return glm::lookAt(position, target, glm::vec3(0.0f, 1.0f, 0.0f));
}

void Camera::setAngles(float theta, float phi)
{
  this->theta = theta;

  // Clamp phi to avoid gimbal lock
  const float epsilon = 0.01f;
  this->phi = glm::clamp(phi, -glm::pi<float>() / 2.0f + epsilon, glm::pi<float>() / 2.0f - epsilon);

  updatePosition();
}

void Camera::adjustDistance(float delta)
{
  distance += delta;

  // Clamp distance to reasonable values
  distance = glm::clamp(distance, 1e5f, 1e9f); // 10,000 km to 1,000,000 km

  updatePosition();
}

void Camera::setAspectRatio(float aspectRatio)
{
  this->aspectRatio = aspectRatio;
  projectionMatrix = glm::perspective(glm::radians(fov), aspectRatio, nearPlane, farPlane);
}
