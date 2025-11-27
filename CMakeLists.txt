cmake_minimum_required(VERSION 3.10)
project(SatelliteConstellation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable optimizations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Add compiler optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  if(MSVC)
    add_compile_options(/O2)
  else()
    add_compile_options(-O3 -march=native)
  endif()
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external)

# Find GLEW
find_package(GLEW REQUIRED)

# Find GLFW
find_package(glfw3 REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLM
find_package(glm REQUIRED)

# Add executable
add_executable(constelation
    src/main.cpp
    src/CelestialBody.cpp
    src/Universe.cpp
    src/Camera.cpp
    src/Sphere.cpp
    src/Cube.cpp
    src/Shader.cpp
    src/Texture.cpp
    src/Satellite.cpp
    src/GroundStation.cpp
    src/LineRenderer.cpp
    src/MathUtils.cpp
)

# Link libraries
target_link_libraries(constelation
    GLEW::GLEW
    glfw
    ${OPENGL_LIBRARIES}
    glm::glm
)

# On macOS, we need to link against the CoreFoundation framework
if (APPLE)
    target_link_libraries(constelation "-framework CoreFoundation")
endif()

# Copy resources to build directory
file(COPY shaders DESTINATION ${CMAKE_BINARY_DIR})
file(COPY textures DESTINATION ${CMAKE_BINARY_DIR})

# Print build info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Project: ${PROJECT_NAME}")
