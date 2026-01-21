cmake_minimum_required(VERSION 3.10)
project(SatelliteConstellation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable optimizations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Platform-specific compiler flags
if(MSVC)
  # MSVC-specific flags
  add_compile_options(/W3)  # Warning level 3
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(/O2)
  endif()
  # Prevent Windows.h from defining min/max macros
  add_compile_definitions(NOMINMAX)
  # Enable math constants like M_PI if needed
  add_compile_definitions(_USE_MATH_DEFINES)
  # Suppress deprecation warnings for standard C functions
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
  # GCC/Clang flags
  add_compile_options(-Wall -Wextra)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
    # Only use -march=native on non-cross-compile builds
    if(NOT CMAKE_CROSSCOMPILING)
      add_compile_options(-march=native)
    endif()
  endif()
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/external)
include_directories(${CMAKE_SOURCE_DIR}/core)
include_directories(${CMAKE_SOURCE_DIR}/dynamics)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/bodies)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/components)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/integrators)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/gravityModels)
include_directories(${CMAKE_SOURCE_DIR}/dynamics/environmentalModels)
include_directories(${CMAKE_SOURCE_DIR}/fsw)
include_directories(${CMAKE_SOURCE_DIR}/fsw/adcs)
include_directories(${CMAKE_SOURCE_DIR}/fsw/power)
include_directories(${CMAKE_SOURCE_DIR}/fsw/stationkeeping)
include_directories(${CMAKE_SOURCE_DIR}/rendering)
include_directories(${CMAKE_SOURCE_DIR}/rendering/primitives)
include_directories(${CMAKE_SOURCE_DIR}/rendering/loaders)
include_directories(${CMAKE_SOURCE_DIR}/data)
include_directories(${CMAKE_SOURCE_DIR}/util)
include_directories(${CMAKE_SOURCE_DIR}/scenarios)

# Fetch ImGui automatically from GitHub
include(FetchContent)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.7  # Specify the version you want
)
FetchContent_MakeAvailable(imgui)

# ImGui setup
set(IMGUI_DIR ${imgui_SOURCE_DIR})
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

# Find OpenGL (available on all platforms)
find_package(OpenGL REQUIRED)

# Find GLEW
# On Windows with vcpkg, GLEW is found automatically
find_package(GLEW REQUIRED)

# Find GLFW
find_package(glfw3 REQUIRED)

# Find GLM
find_package(glm REQUIRED)

# Add executable
add_executable(simulation
    # Core
    core/main.cpp
    core/Simulation.cpp

    # Dynamics
    dynamics/bodies/Spacecraft.cpp
    dynamics/bodies/CelestialBody.cpp
    dynamics/bodies/GroundStation.cpp
    dynamics/Universe.cpp
    dynamics/Orbit.cpp

    # Dynamics - Integrators
    dynamics/integrators/RK4Integrator.cpp

    # Dynamics - Gravity Models
    dynamics/gravityModels/GravityModel.cpp

    # Dynamics - Environmental Models
    dynamics/environmentalModels/EnvironmentalModels.cpp

    # Urils
    util/MathUtils.cpp

    # Components
    dynamics/components/IMU.cpp
    dynamics/components/SolarPanel.cpp
    dynamics/components/GPS.cpp
    dynamics/components/ReactionWheel.cpp
    dynamics/components/SunSensor.cpp
    dynamics/components/Magnetometer.cpp
    dynamics/components/Magnetorquer.cpp
    dynamics/components/Thruster.cpp
    dynamics/components/CMG.cpp

    # Scenarios
    scenarios/SSOTestScenario.cpp
    scenarios/GPSScenario.cpp
    scenarios/StarlinkScenario.cpp
    scenarios/GEOScenario.cpp
    scenarios/MolniyaScenario.cpp

    # Flight Software
    fsw/FlightSoftware.cpp
    fsw/PIDController.cpp
    fsw/adcs/ADCSController.cpp
    fsw/adcs/AttitudeEstimator.cpp

    # Rendering
    rendering/Renderer.cpp
    rendering/Shader.cpp
    rendering/Texture.cpp
    rendering/loaders/OBJMesh.cpp
    rendering/primitives/Sphere.cpp
    rendering/primitives/Cube.cpp
    rendering/primitives/LineRenderer.cpp

    # GUI
    rendering/GUI.cpp
    rendering/Camera.cpp

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
    OpenGL::GL
    glm::glm
)

# Platform-specific linking
if(APPLE)
    # macOS frameworks
    target_link_libraries(simulation "-framework CoreFoundation")
elseif(WIN32)
    # Windows-specific libraries (if needed)
    # OpenGL32 is automatically linked via OpenGL::GL
endif()

# Copy resources to build directory
file(COPY rendering/shaders DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources/textures DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources/models DESTINATION ${CMAKE_BINARY_DIR})

# On Windows with shared libraries, copy DLLs to output directory
if(WIN32)
    # For multi-config generators (Visual Studio), copy to both Debug and Release
    if(CMAKE_CONFIGURATION_TYPES)
        set(OUTPUT_DIRS ${CMAKE_BINARY_DIR}/Debug ${CMAKE_BINARY_DIR}/Release)
    else()
        set(OUTPUT_DIRS ${CMAKE_BINARY_DIR})
    endif()

    # Copy resource directories to all output directories for multi-config builds
    foreach(OUTPUT_DIR ${OUTPUT_DIRS})
        file(COPY rendering/shaders DESTINATION ${OUTPUT_DIR})
        file(COPY resources/textures DESTINATION ${OUTPUT_DIR})
        file(COPY resources/models DESTINATION ${OUTPUT_DIR})
    endforeach()
endif()

# Print build info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
