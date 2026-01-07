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

# ImGui setup
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

# Find GLEW
find_package(GLEW REQUIRED)

# Find GLFW
find_package(glfw3 REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLM
find_package(glm REQUIRED)

# Add executable
add_executable(simulation
    src/main.cpp
    src/Simulation.cpp
    src/GUI.cpp
    src/CelestialBody.cpp
    src/Universe.cpp
    src/Camera.cpp
    src/Sphere.cpp
    src/Cube.cpp
    src/Shader.cpp
    src/Texture.cpp
    src/Satellite.cpp
    src/GroundStation.cpp
    src/CommLink.cpp
    src/LineRenderer.cpp
    src/MathUtils.cpp
    src/Renderer.cpp
    src/Orbit.cpp
    src/OBJMesh.cpp

    # Flight Software
    src/StandardFSW.cpp
    src/PassiveFSW.cpp

    # ImGui core
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp

    # ImGui backends
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Link libraries
target_link_libraries(simulation
    GLEW::GLEW
    glfw
    ${OPENGL_LIBRARIES}
    glm::glm
)

# On macOS, we need to link against the CoreFoundation framework
if (APPLE)
    target_link_libraries(simulation "-framework CoreFoundation")
endif()

# Copy resources to build directory
file(COPY shaders DESTINATION ${CMAKE_BINARY_DIR})
file(COPY textures DESTINATION ${CMAKE_BINARY_DIR})
file(COPY models DESTINATION ${CMAKE_BINARY_DIR})

# Print build info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Project: ${PROJECT_NAME}")
