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
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/external)
include_directories(${CMAKE_SOURCE_DIR}/core)
include_directories(${CMAKE_SOURCE_DIR}/dynamics)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/integrators)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/gravityModels)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/environmentalModels)
include_directories(${CMAKE_SOURCE_DIR}/sensors)
include_directories(${CMAKE_SOURCE_DIR}/fsw)
include_directories(${CMAKE_SOURCE_DIR}/fsw/adcs)
include_directories(${CMAKE_SOURCE_DIR}/fsw/power)
include_directories(${CMAKE_SOURCE_DIR}/fsw/stationkeeping)
include_directories(${CMAKE_SOURCE_DIR}/rendering)
include_directories(${CMAKE_SOURCE_DIR}/rendering/primitives)
include_directories(${CMAKE_SOURCE_DIR}/gui)
include_directories(${CMAKE_SOURCE_DIR}/infrastructure)
include_directories(${CMAKE_SOURCE_DIR}/data)
include_directories(${CMAKE_SOURCE_DIR}/util)

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
    # Core
    core/main.cpp
    core/Simulation.cpp

    # Dynamics
    dynamics/CelestialBody.cpp
    dynamics/Universe.cpp
    dynamics/Satellite.cpp
    dynamics/Orbit.cpp

    # Dynamics - Integrators
    dynamics/integrators/RK4Integrator.cpp

    # Dynamics - Gravity Models
    dynamics/gravityModels/GravityModel.cpp

    # Dynamics - Environmental Models
    dynamics/environmentalModels/EnvironmentalModels.cpp

    # Urils
    util/MathUtils.cpp

    # Sensors
    sensors/IMU.cpp

    # Flight Software - ADCS
    fsw/adcs/ADCSController.cpp
    fsw/adcs/StandardFSW.cpp
    fsw/adcs/PassiveFSW.cpp

    # Flight Software - Power
    fsw/power/PowerManager.cpp

    # Flight Software - Station Keeping
    fsw/stationkeeping/StationKeepingController.cpp

    # Rendering
    rendering/Renderer.cpp
    rendering/Shader.cpp
    rendering/Texture.cpp
    rendering/OBJMesh.cpp
    rendering/primitives/Sphere.cpp
    rendering/primitives/Cube.cpp
    rendering/primitives/LineRenderer.cpp

    # GUI
    gui/GUI.cpp
    gui/Camera.cpp

    # Infrastructure
    infrastructure/GroundStation.cpp
    infrastructure/CommLink.cpp

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
file(COPY resources/shaders DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources/textures DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources/models DESTINATION ${CMAKE_BINARY_DIR})

# Print build info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Project: ${PROJECT_NAME}")
