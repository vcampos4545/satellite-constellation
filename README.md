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
- **ImGui**: UI rendering (automatically fetched via CMake)

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
./simulation
```

### Build Options

```bash
# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build (default, optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

### ImGui (Dear ImGui)

The project uses [Dear ImGui](https://github.com/ocornut/imgui) for the user interface. **No manual installation is required** - CMake automatically downloads ImGui from GitHub during the build process using FetchContent.

If you need to use a specific version of ImGui, you can modify the `GIT_TAG` in `CMakeLists.txt`:

```cmake
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.7  # Change this to your desired version
)
```

## Coordinate System

### Earth-centered inertial (ECI)

<p align="center" width="100%">
<image src="https://upload.wikimedia.org/wikipedia/commons/3/32/Earth_Centered_Inertial_Coordinate_System.png" width="80%">
</p>

- Z: Up (North Pole)
- X: vernal equinox (0 degress longitude)
- Y: Complete right hand rule
- Note: XY Plane along equatorial plane, Earth not fixed (rotating)

## Controls

### Camera

- **Mouse Drag (Left Click)**: Rotate camera around Earth
- **Mouse Scroll**: Zoom in/out

### Time Control

- **SPACE**: Pause/Resume simulation
- **. or +**: Increase time warp (1x → 10x → 100x → 1000x → 10000x)
- **, or -**: Decrease time warp

## Credits

- Texture loading via stb_image by Sean Barrett

## License

This project is for educational and simulation purposes.
