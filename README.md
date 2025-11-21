# Satellite Constellation Simulation

A realistic 3D simulation of satellite constellations orbiting Earth, featuring accurate orbital mechanics, Starlink-like LEO mesh patterns, and real-time physics.

## Features

- **Realistic Orbital Mechanics**: Uses Newtonian gravity with proper physics integration
- **Starlink-like Constellation**: 96 satellites in LEO with 53° inclination across 8 orbital planes
- **Complete Orbital Paths**: Full orbit visualization showing satellite trajectories
- **Textured Earth**: Support for Earth day textures with proper sun lighting
- **Time Warp**: Speed up or slow down time (1x to 10000x) with stable physics sub-stepping
- **Interactive Camera**: Pan, zoom, and rotate around Earth
- **Pause/Resume**: Freeze the simulation at any time
- **Realistic Scale**: Accurate masses, radii, and distances for Earth, Sun, and satellites

## Project Structure

```
satelitte-constelation/
├── include/          # Header files (.h)
├── src/              # Implementation files (.cpp)
├── shaders/          # GLSL shaders (.vert, .frag)
├── textures/         # Earth texture assets
├── external/         # Third-party libraries (stb_image.h)
├── build/            # Build output (gitignored)
├── CMakeLists.txt    # CMake build configuration
├── .gitignore
└── README.md
```

## Dependencies

### Required Libraries
- **OpenGL 3.3+**: Graphics rendering
- **GLFW3**: Window and input management
- **GLEW**: OpenGL extension loading
- **GLM**: OpenGL Mathematics library

### macOS Installation (Homebrew)
```bash
brew install glfw glew glm
```

### Linux Installation (Ubuntu/Debian)
```bash
sudo apt-get install libglfw3-dev libglew-dev libglm-dev
```

## Building

### macOS/Linux
```bash
# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build
make

# Run
./constelation
```

### Build Options
```bash
# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build (default, optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

## Earth Texture Setup

1. Download an Earth texture from:
   - [Solar System Scope](https://www.solarsystemscope.com/textures/) (recommended)
   - [NASA Visible Earth](https://visibleearth.nasa.gov/)

2. Save as `textures/earth.jpg`

3. The simulation will automatically load it on startup

See `textures/README.md` for more details.

## Controls

### Camera
- **Mouse Drag (Left Click)**: Rotate camera around Earth
- **Mouse Scroll**: Zoom in/out

### Time Control
- **SPACE**: Pause/Resume simulation
- **. or +**: Increase time warp (1x → 10x → 100x → 1000x → 10000x)
- **, or -**: Decrease time warp

### Visual Feedback
- Window title shows current time warp multiplier: `[100x]`
- Console prints time warp changes and pause status

## Architecture

### Core Classes

**Universe** (`Universe.h/cpp`)
- Manages all celestial bodies and satellites
- Handles physics updates with sub-stepping
- Creates satellite constellations

**CelestialBody** (`CelestialBody.h/cpp`)
- Represents Earth, Sun, and other celestial objects
- Stores position, mass, radius, rotation

**Satellite** (`Satellite.h/cpp`)
- Individual satellite with position and velocity
- Physics integration (gravitational acceleration)
- Full orbit path calculation

**Camera** (`Camera.h/cpp`)
- Orbital camera with spherical coordinates
- Perspective projection
- Zoom and pan controls

### Rendering Components

**Sphere** (`Sphere.h/cpp`)
- Procedural sphere mesh generation
- Supports texture coordinates and normals

**LineRenderer** (`LineRenderer.h/cpp`)
- Renders orbit paths as GL_LINE_STRIP

**Shader** (`Shader.h/cpp`)
- GLSL shader loading and management
- Uniform setters for lighting and transforms

**Texture** (`Texture.h/cpp`)
- Image loading via stb_image
- OpenGL texture binding

## Physics

### Orbital Mechanics
- **Gravitational Force**: F = G × M × m / r²
- **Integration**: Euler method with adaptive sub-stepping
- **Stability**: Maximum 0.1s physics steps, regardless of time warp

### Sub-Stepping Algorithm
```
Real time step (60 FPS): 0.016s
Time warp: 1000x
Warped time: 16s
Physics substeps: 160 iterations × 0.1s
Result: Stable circular orbits
```

### Orbital Parameters
- **LEO Altitude**: 550 km (Starlink-like)
- **LEO Inclination**: 53°
- **LEO Orbital Period**: ~95 minutes
- **GEO Altitude**: 35,786 km
- **GEO Orbital Period**: 24 hours

## Performance

- **96 satellites**: ~60 FPS on modern hardware
- **Physics updates**: Sub-stepped for stability
- **Rendering**: OpenGL 3.3 core profile
- **Line smoothing**: Enabled for orbit paths

## Known Limitations

- Euler integration (not Runge-Kutta) for simplicity
- No satellite-satellite interactions
- Earth rotation doesn't affect satellite orbits (simplified)
- No atmospheric drag simulation

## Future Enhancements

- [ ] Add more constellation patterns (polar, sun-synchronous)
- [ ] Implement ground station visualization
- [ ] Add satellite-to-ground coverage visualization
- [ ] Support for elliptical orbits
- [ ] Multi-threaded physics updates
- [ ] ImGui control panel
- [ ] Save/load simulation states

## License

This project is for educational and simulation purposes.

## Credits

- Physics based on Newtonian mechanics
- Starlink orbital parameters from SpaceX data
- Texture loading via stb_image by Sean Barrett
