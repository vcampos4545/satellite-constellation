#include "Camera.h"
#include <glm/gtc/constants.hpp>

Camera::Camera(float fov, float aspectRatio, float nearPlane, float farPlane)
    : fov(fov), aspectRatio(aspectRatio), nearPlane(nearPlane), farPlane(farPlane),
      target(0.0f, 0.0f, 0.0f), distance(2e7f), theta(0.0f), phi(0.0f)
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

void Camera::screenToWorldRay(double mouseX, double mouseY, int screenWidth, int screenHeight,
                              glm::vec3 &rayOrigin, glm::vec3 &rayDirection) const
{
  // Convert mouse coordinates from screen space to normalized device coordinates (NDC)
  // Screen space: (0,0) is top-left, (width, height) is bottom-right
  // NDC: (-1,-1) is bottom-left, (1,1) is top-right
  float x = (2.0f * mouseX) / screenWidth - 1.0f;
  float y = 1.0f - (2.0f * mouseY) / screenHeight; // Flip Y axis

  // NDC coordinates
  glm::vec4 rayStartNDC(x, y, -1.0f, 1.0f); // Near plane
  glm::vec4 rayEndNDC(x, y, 1.0f, 1.0f);    // Far plane

  // Convert to clip space (apply inverse projection)
  glm::mat4 invProjection = glm::inverse(projectionMatrix);
  glm::vec4 rayStartClip = invProjection * rayStartNDC;
  glm::vec4 rayEndClip = invProjection * rayEndNDC;

  // Convert to eye space (perspective divide)
  rayStartClip /= rayStartClip.w;
  rayEndClip /= rayEndClip.w;

  // Convert to world space (apply inverse view)
  glm::mat4 invView = glm::inverse(getViewMatrix());
  glm::vec4 rayStartWorld = invView * rayStartClip;
  glm::vec4 rayEndWorld = invView * rayEndClip;

  // Extract origin and direction
  rayOrigin = glm::vec3(rayStartWorld);
  rayDirection = glm::normalize(glm::vec3(rayEndWorld) - rayOrigin);
}
