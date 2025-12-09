# Satellite Constellation Simulation

A satellite constellation simulation.

<p align="center" width="100%">
<video src="https://github.com/user-attachments/assets/6b83de09-5c4b-4eb9-91e8-3541c2c4ee0f" width="80%" controls></video>
</p>

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

## License

This project is for educational and simulation purposes.

## Credits

- Texture loading via stb_image by Sean Barrett
