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

## Physics & Mathematics

This simulation implements a high-fidelity orbital mechanics and attitude control system with multiple environmental perturbations.

### Orbital Mechanics

#### Gravitational Forces

**Earth Gravity (Two-Body Problem)**
```
a_earth = -μ_earth * r / |r|³
where μ_earth = G * M_earth = 3.986 × 10¹⁴ m³/s²
```

**Third-Body Perturbations (Sun and Moon)**
```
a_sun = G * M_sun * (r_sun - r_sat) / |r_sun - r_sat|³
a_moon = G * M_moon * (r_moon - r_sat) / |r_moon - r_sat|³
```

#### Atmospheric Drag

Exponential atmosphere model with drag force opposing velocity:

```
ρ(h) = ρ₀ * e^(-h/H)
where:
  ρ₀ = 1.225 kg/m³ (sea level density)
  H = 8500 m (scale height)
  h = altitude above Earth surface

F_drag = -½ * ρ(h) * v² * C_d * A * v̂
a_drag = F_drag / m

where:
  C_d = 2.2 (drag coefficient)
  A = 10 m² (cross-sectional area)
  m = 260 kg (satellite mass)
  v̂ = velocity direction unit vector
```

#### Solar Radiation Pressure

Radiation pressure from sunlight (only when satellite is illuminated):

```
P(r) = P₀ * (AU / r)²
where P₀ = I_solar / c = 1367 / 299792458 = 4.56 × 10⁻⁶ N/m²

F_srp = P(r) * A * C_r * ŝ
a_srp = F_srp / m

where:
  C_r = 1.3 (reflectivity coefficient: 1.0 = absorbing, 2.0 = perfect mirror)
  ŝ = sun direction unit vector

Eclipse Model: Cylindrical shadow approximation
```

#### Orbital Elements & Initialization

**Circular Orbits (Starlink Constellation)**
```
Orbital radius: r = R_earth + h
Orbital velocity: v = √(μ/r)
Inclination: i = 53° (typical Starlink)
RAAN (Right Ascension of Ascending Node): Ω = 2π * planeId / numPlanes
True Anomaly: ν = 2π * satId / satsPerPlane
```

**Elliptical Orbits (Molniya Constellation)**
```
Semi-major axis: a = 26.6 × 10⁶ m
Eccentricity: e = 0.72
Inclination: i = 63.4° (critical inclination - minimizes apsidal precession)
Argument of perigee: ω = 270° (apogee over northern hemisphere)

Radius: r(ν) = a(1 - e²) / (1 + e*cos(ν))
Velocity magnitude: v = √(μ/p) where p = a(1 - e²)
Velocity direction: v_r = √(μ/p) * e*sin(ν), v_θ = √(μ/p) * (1 + e*cos(ν))
```

**Orbital Period (Kepler's Third Law)**
```
T = 2π√(a³/μ)

Specific orbital energy: ε = v²/2 - μ/r
Semi-major axis from energy: a = -μ/(2ε)
```

#### Numerical Integration

**RK4 (Runge-Kutta 4th Order) - Orbital Dynamics**

The simulation uses RK4 for orbital position and velocity integration:

```
k₁ = f(t, y)
k₂ = f(t + dt/2, y + k₁*dt/2)
k₃ = f(t + dt/2, y + k₂*dt/2)
k₄ = f(t + dt, y + k₃*dt)

y_new = y + (dt/6) * (k₁ + 2k₂ + 2k₃ + k₄)

where:
  y = [position, velocity]
  f(t, y) = [velocity, acceleration]
```

**Sub-Stepping for Stability**

Large time warps are broken into smaller physics steps:

```
Real time step (60 FPS): 0.016s
Time warp: 1000x
Warped time: 16s
Physics substeps: 160 iterations × 0.1s max
Result: Stable orbits with minimal numerical drift
```

### Attitude Determination and Control System (ADCS)

The satellite implements a complete ADCS control loop mirroring flight software architecture.

#### ADCS Control Loop Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      ADCS CONTROL LOOP (10-100 Hz)                      │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 1: ATTITUDE DETERMINATION                   │
        │  ─────────────────────────────────                │
        │  • Gyroscopes (high rate, drift)                  │
        │  • Star Trackers (absolute, slow)                 │
        │  • Sun Sensors (1-2 axis)                         │
        │  • Magnetometers (2 axis)                         │
        │  → Kalman Filter → q_current, ω_current           │
        │                                                    │
        │  Simulation: Perfect knowledge (q, ω)             │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 2: COMPUTE TARGET ATTITUDE                  │
        │  ────────────────────────────                     │
        │  Mode Selection:                                  │
        │  • NADIR_POINTING: q_target aligns Z → -r̂        │
        │  • SUN_POINTING: q_target aligns Z → sun          │
        │  • VELOCITY_POINTING: q_target aligns X → v̂      │
        │  • TARGET_TRACKING: q_target aligns Z → target    │
        │  • INERTIAL_HOLD: q_target = const                │
        │  • DETUMBLE: minimize |ω|                         │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 3: COMPUTE ATTITUDE ERROR                   │
        │  ───────────────────────────                      │
        │  q_error = q_target ⊗ q_current⁻¹                 │
        │                                                    │
        │  Convert to axis-angle:                           │
        │  θ_error = 2*atan2(|[qx,qy,qz]|, qw)              │
        │  e = normalize([qx,qy,qz]) * θ_error              │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 4: PID CONTROLLER                           │
        │  ───────────────────                              │
        │  τ = Kp*e + Ki*∫e*dt + Kd*(-ω)                    │
        │                                                    │
        │  Proportional: Kp*e (stiffness)                   │
        │  Integral: Ki*∫e (eliminate steady-state error)   │
        │  Derivative: -Kd*ω (damping)                      │
        │                                                    │
        │  Auto-tuning (based on desired settling time):    │
        │  ωn = 4/(ζ*ts), Kp = I*ωn², Kd = 2*ζ*I*ωn        │
        │  Ki = 0.01*Kp (with anti-windup)                  │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 5: ACTUATOR ALLOCATION                      │
        │  ────────────────────────                         │
        │  ┌─────────────────────────────────────┐          │
        │  │  Reaction Wheels (primary)          │          │
        │  │  τ_rw = commanded torque            │          │
        │  │  h_rw += τ_rw * dt                  │          │
        │  │  Saturation: |h_rw| < h_max         │          │
        │  │  → Desaturation with magnetorquers  │          │
        │  └─────────────────────────────────────┘          │
        │  ┌─────────────────────────────────────┐          │
        │  │  Magnetorquers (desaturation)       │          │
        │  │  B-dot control: m = -k*(ω × B)      │          │
        │  │  τ_mag = m × B                      │          │
        │  │  (perpendicular to B-field only)    │          │
        │  └─────────────────────────────────────┘          │
        │  ┌─────────────────────────────────────┐          │
        │  │  CMGs (high-performance missions)   │          │
        │  │  Gimbal steering laws               │          │
        │  │  Singularity avoidance              │          │
        │  └─────────────────────────────────────┘          │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 6: ATTITUDE DYNAMICS PROPAGATION            │
        │  ──────────────────────────────────────           │
        │  Euler's Rigid Body Equations:                    │
        │  I*α = τ_external - ω × (I*ω)                     │
        │                                                    │
        │  Quaternion Kinematics:                           │
        │  q̇ = ½*Ω(ω)*q = ½*[0,ω]⊗q                        │
        │                                                    │
        │  Integration (Euler method):                      │
        │  ω_new = ω + α*dt                                 │
        │  q_new = q + q̇*dt                                 │
        │  q_new = normalize(q_new)                         │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────┐
                    │   Update Complete         │
                    │   Return to Step 1        │
                    └───────────────────────────┘
```

#### Detailed ADCS Mathematics

**Attitude Representation (Quaternions)**

```
Quaternion: q = [qw, qx, qy, qz] = [cos(θ/2), sin(θ/2)*axis]

Quaternion multiplication (rotation composition):
q₁ ⊗ q₂ = [w₁w₂ - v₁·v₂, w₁v₂ + w₂v₁ + v₁×v₂]

Inverse rotation: q⁻¹ = [qw, -qx, -qy, -qz] (for unit quaternions)

Rotate vector: v' = q ⊗ [0,v] ⊗ q⁻¹
```

**Quaternion Kinematics**

```
q̇ = ½ * Ω(ω) * q

where Ω(ω) = [ 0   -ωx  -ωy  -ωz ]
             [ ωx   0    ωz  -ωy ]
             [ ωy  -ωz   0    ωx ]
             [ ωz   ωy  -ωx   0  ]

Simplified: q̇ = ½ * [0, ω] ⊗ q
```

**Euler's Rigid Body Equations**

```
I*α = τ_external - ω × (I*ω)

where:
  I = [Ixx, Iyy, Izz] = [50, 50, 20] kg·m² (diagonal inertia tensor)
  α = dω/dt (angular acceleration)
  ω = angular velocity vector
  τ_external = control torque from actuators

Gyroscopic torque: τ_gyro = -ω × (I*ω)
```

**PID Control Law**

```
τ_control = Kp*e_att + Ki*∫e_att*dt + Kd*(-ω)

Proportional gain: Kp = I*ωn² (N·m/rad)
Derivative gain: Kd = 2*ζ*I*ωn (N·m·s/rad)
Integral gain: Ki = 0.01*Kp (N·m/(rad·s))

where:
  ωn = natural frequency = 4/(ζ*ts)
  ζ = damping ratio (typically 0.7-0.9)
  ts = settling time (typically 20-60 seconds)

Anti-windup: |∫e_att*dt| < e_max
```

**Reaction Wheel Dynamics**

```
Torque application: τ_spacecraft = -τ_wheel (Newton's 3rd law)

Momentum accumulation: h_wheel = ∫τ_wheel*dt

Constraints:
  |τ_wheel| < τ_max = 0.1 N·m (per wheel)
  |h_wheel| < h_max = 10 N·m·s (per wheel)

When saturated: Use magnetorquers for desaturation
```

**Magnetorquer B-dot Control**

```
Detumble mode (remove initial tumbling):
dB/dt ≈ ω × B (in body frame)
m = -k * (ω × B)  (dipole moment command)
τ = m × B  (generated torque)

Note: Can only generate torque perpendicular to B-field
Effective for slow detumbling, not precision pointing
```

#### Control Modes

| Mode | Description | Target Alignment | Use Case |
|------|-------------|------------------|----------|
| **NADIR_POINTING** | Z-axis points to Earth center | Z-axis → -r̂ | Earth observation, communications |
| **SUN_POINTING** | Z-axis points to Sun | Z-axis → sun | Solar panel alignment |
| **VELOCITY_POINTING** | X-axis along velocity | X-axis → v̂ | Aerodynamic stability |
| **TARGET_TRACKING** | Z-axis points to ground target | Z-axis → target | Ground station comms |
| **INERTIAL_HOLD** | Maintain fixed orientation | q = constant | Star tracking, astronomy |
| **DETUMBLE** | Reduce angular velocity | minimize \|ω\| | Post-deployment stabilization |

### Footprint Calculation

The satellite's ground coverage footprint is computed geometrically:

```
Horizon angle: λ₀ = arccos(R_earth / r_sat)

For each point on footprint circle:
  1. Start with nadir direction: n̂ = (sat - earth) / |sat - earth|
  2. Find perpendicular vectors forming horizon plane
  3. Rotate horizon vector around nadir axis: θ ∈ [0, 2π]
  4. Project to Earth surface

Footprint radius on surface: d = R_earth * arcsin(sin(λ₀) * r_sat / R_earth)
```

## License

This project is for educational and simulation purposes.

## Credits

- Texture loading via stb_image by Sean Barrett
