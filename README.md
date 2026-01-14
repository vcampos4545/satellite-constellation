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

## Physics & Mathematics

This simulation implements a high-fidelity orbital mechanics and attitude control system with multiple environmental perturbations.

### Orbital Mechanics

#### Gravitational Forces

**Earth Gravity (Two-Body Problem)**

$$\vec{a}_{\text{earth}} = -\frac{\mu_{\text{earth}} \vec{r}}{|\vec{r}|^3}$$

where $\mu_{\text{earth}} = G M_{\text{earth}} = 3.986 \times 10^{14} \text{ m}^3/\text{s}^2$

**Third-Body Perturbations (Sun and Moon)**

$$\vec{a}_{\text{sun}} = \frac{G M_{\text{sun}} (\vec{r}_{\text{sun}} - \vec{r}_{\text{sat}})}{|\vec{r}_{\text{sun}} - \vec{r}_{\text{sat}}|^3}$$

$$\vec{a}_{\text{moon}} = \frac{G M_{\text{moon}} (\vec{r}_{\text{moon}} - \vec{r}_{\text{sat}})}{|\vec{r}_{\text{moon}} - \vec{r}_{\text{sat}}|^3}$$

#### Atmospheric Drag

Exponential atmosphere model with drag force opposing velocity:

$$\rho(h) = \rho_0 e^{-h/H}$$

where:

- $\rho_0 = 1.225 \text{ kg/m}^3$ (sea level density)
- $H = 8500 \text{ m}$ (scale height)
- $h$ = altitude above Earth surface

$$\vec{F}_{\text{drag}} = -\frac{1}{2} \rho(h) v^2 C_d A \hat{v}$$

$$\vec{a}_{\text{drag}} = \frac{\vec{F}_{\text{drag}}}{m}$$

where:

- $C_d = 2.2$ (drag coefficient)
- $A = 10 \text{ m}^2$ (cross-sectional area)
- $m = 260 \text{ kg}$ (satellite mass)
- $\hat{v}$ = velocity direction unit vector

#### Solar Radiation Pressure

Radiation pressure from sunlight (only when satellite is illuminated):

$$P(r) = P_0 \left(\frac{\text{AU}}{r}\right)^2$$

where $P_0 = \frac{I_{\text{solar}}}{c} = \frac{1367}{299792458} = 4.56 \times 10^{-6} \text{ N/m}^2$

$$\vec{F}_{\text{srp}} = P(r) A C_r \hat{s}$$

$$\vec{a}_{\text{srp}} = \frac{\vec{F}_{\text{srp}}}{m}$$

where:

- $C_r = 1.3$ (reflectivity coefficient: 1.0 = absorbing, 2.0 = perfect mirror)
- $\hat{s}$ = sun direction unit vector
- Eclipse Model: Cylindrical shadow approximation

### Attitude Determination and Control System (ADCS)

The satellite implements a complete ADCS control loop mirroring flight software architecture.

#### Detailed ADCS Mathematics

**Attitude Representation (Quaternions)**

$$q = [q_w, q_x, q_y, q_z] = \left[\cos\left(\frac{\theta}{2}\right), \sin\left(\frac{\theta}{2}\right)\vec{u}\right]$$

Quaternion multiplication (rotation composition):

$$q_1 \otimes q_2 = [w_1 w_2 - \vec{v}_1 \cdot \vec{v}_2, \, w_1 \vec{v}_2 + w_2 \vec{v}_1 + \vec{v}_1 \times \vec{v}_2]$$

Inverse rotation: $q^{-1} = [q_w, -q_x, -q_y, -q_z]$ (for unit quaternions)

Rotate vector: $\vec{v}' = q \otimes [0, \vec{v}] \otimes q^{-1}$

**Quaternion Kinematics**

$$\dot{q} = \frac{1}{2} \Omega(\vec{\omega}) q$$

where

$$
\Omega(\vec{\omega}) = \begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

Simplified: $\dot{q} = \frac{1}{2} [0, \vec{\omega}] \otimes q$

**Euler's Rigid Body Equations**

$$I \vec{\alpha} = \vec{\tau}_{\text{external}} - \vec{\omega} \times (I\vec{\omega})$$

where:

- $I = [I_{xx}, I_{yy}, I_{zz}] = [50, 50, 20] \text{ kg·m}^2$ (diagonal inertia tensor)
- $\vec{\alpha} = \frac{d\vec{\omega}}{dt}$ (angular acceleration)
- $\vec{\omega}$ = angular velocity vector
- $\vec{\tau}_{\text{external}}$ = control torque from actuators

Gyroscopic torque: $\vec{\tau}_{\text{gyro}} = -\vec{\omega} \times (I\vec{\omega})$

## Credits

- Texture loading via stb_image by Sean Barrett

## License

This project is for educational and simulation purposes.
