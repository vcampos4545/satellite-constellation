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
./simulation
```

### Build Options

```bash
# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build (default, optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

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
        │                                                   │
        │  Simulation: Perfect knowledge (q, ω)             │
        └───────────────────────────────────────────────────┘
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────┐
        │  STEP 2: COMPUTE TARGET ATTITUDE                  │
        │  ────────────────────────────                     │
        │  Mode Selection:                                  │
        │  • NADIR_POINTING: q_target aligns Z → -r̂         │
        │  • SUN_POINTING: q_target aligns Z → sun          │
        │  • VELOCITY_POINTING: q_target aligns X → v̂       │
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
        │                                                   │
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
        │                                                   │
        │  Proportional: Kp*e (stiffness)                   │
        │  Integral: Ki*∫e (eliminate steady-state error)   │
        │  Derivative: -Kd*ω (damping)                      │
        │                                                   │
        │  Auto-tuning (based on desired settling time):    │
        │  ωn = 4/(ζ*ts), Kp = I*ωn², Kd = 2*ζ*I*ωn         │
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
        │                                                   │
        │  Quaternion Kinematics:                           │
        │  q̇ = ½*Ω(ω)*q = ½*[0,ω]⊗q                         │
        │                                                   │
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

**PID Control Law**

$$\vec{\tau}_{\text{control}} = K_p \vec{e}_{\text{att}} + K_i \int \vec{e}_{\text{att}} \, dt + K_d (-\vec{\omega})$$

Auto-tuned gains:

- Proportional gain: $K_p = I \omega_n^2$ (N·m/rad)
- Derivative gain: $K_d = 2\zeta I \omega_n$ (N·m·s/rad)
- Integral gain: $K_i = 0.01 K_p$ (N·m/(rad·s))

where:

- $\omega_n = \frac{4}{\zeta t_s}$ (natural frequency)
- $\zeta$ = 0.7-0.9 (damping ratio)
- $t_s$ = 20-60 s (settling time)
- Anti-windup: $|\int \vec{e}_{\text{att}} \, dt| < e_{\max}$

**Reaction Wheel Dynamics**

$$\vec{\tau}_{\text{spacecraft}} = -\vec{\tau}_{\text{wheel}} \quad \text{(Newton's 3rd law)}$$

$$\vec{h}_{\text{wheel}} = \int \vec{\tau}_{\text{wheel}} \, dt \quad \text{(momentum accumulation)}$$

Constraints:

- $|\tau_{\text{wheel}}| < \tau_{\max} = 0.1 \text{ N·m}$ (per wheel)
- $|h_{\text{wheel}}| < h_{\max} = 10 \text{ N·m·s}$ (per wheel)
- When saturated: Use magnetorquers for desaturation

**Magnetorquer B-dot Control**

Detumble mode (remove initial tumbling):

$$\frac{d\vec{B}}{dt} \approx \vec{\omega} \times \vec{B} \quad \text{(in body frame)}$$

$$\vec{m} = -k (\vec{\omega} \times \vec{B}) \quad \text{(dipole moment command)}$$

$$\vec{\tau} = \vec{m} \times \vec{B} \quad \text{(generated torque)}$$

Note: Can only generate torque perpendicular to $\vec{B}$-field. Effective for slow detumbling, not precision pointing.

**LQR Control (Linear Quadratic Regulator)** _(Available in simulation)_

Optimal control minimizing cost function:

$$J = \int_0^\infty (\vec{x}^T Q \vec{x} + \vec{u}^T R \vec{u}) \, dt$$

where $\vec{x} = [\vec{e}_{\text{att}}, \vec{\omega}]$ (state), $\vec{u} = \vec{\tau}$ (control input)

Control law: $\vec{u} = -K \vec{x}$ where $K$ is computed from algebraic Riccati equation.

**MPC Control (Model Predictive Control)** _(Available in simulation)_

Solves optimization problem at each timestep over prediction horizon:

$$\min_{\vec{u}(t)} \sum_{k=0}^{N} \left(\|\vec{x}(k)\|_Q^2 + \|\vec{u}(k)\|_R^2\right)$$

subject to:

- Dynamics: $\vec{x}(k+1) = f(\vec{x}(k), \vec{u}(k))$
- Constraints: $|\vec{\tau}| \leq \tau_{\max}$, $|\vec{h}_{\text{wheel}}| \leq h_{\max}$

#### Control Modes

| Mode                  | Description                    | Target Alignment | Use Case                          |
| --------------------- | ------------------------------ | ---------------- | --------------------------------- |
| **NADIR_POINTING**    | Z-axis points to Earth center  | Z-axis → -r̂      | Earth observation, communications |
| **SUN_POINTING**      | Z-axis points to Sun           | Z-axis → sun     | Solar panel alignment             |
| **VELOCITY_POINTING** | X-axis along velocity          | X-axis → v̂       | Aerodynamic stability             |
| **TARGET_TRACKING**   | Z-axis points to ground target | Z-axis → target  | Ground station comms              |
| **INERTIAL_HOLD**     | Maintain fixed orientation     | q = constant     | Star tracking, astronomy          |
| **DETUMBLE**          | Reduce angular velocity        | minimize \|ω\|   | Post-deployment stabilization     |

### Footprint Calculation

The satellite's ground coverage footprint is computed geometrically:

$$\lambda_0 = \arccos\left(\frac{R_{\text{earth}}}{r_{\text{sat}}}\right) \quad \text{(horizon angle)}$$

For each point on footprint circle:

1. Start with nadir direction: $\hat{n} = \frac{\vec{r}_{\text{sat}} - \vec{r}_{\text{earth}}}{|\vec{r}_{\text{sat}} - \vec{r}_{\text{earth}}|}$
2. Find perpendicular vectors forming horizon plane
3. Rotate horizon vector around nadir axis: $\theta \in [0, 2\pi]$
4. Project to Earth surface

$$d = R_{\text{earth}} \cdot \arcsin\left(\sin(\lambda_0) \cdot \frac{r_{\text{sat}}}{R_{\text{earth}}}\right) \quad \text{(footprint radius)}$$

## License

This project is for educational and simulation purposes.

## Credits

- Texture loading via stb_image by Sean Barrett

## Texture Setup

### Earth Texture

1. Download an Earth texture from:

   - [Solar System Scope](https://www.solarsystemscope.com/textures/) (recommended)
   - [NASA Visible Earth](https://visibleearth.nasa.gov/)

2. Save as `textures/earth.jpg`

3. The simulation will automatically load it on startup

### Moon Texture

1. Download a Moon texture from:

   - [Solar System Scope](https://www.solarsystemscope.com/textures/) (recommended)
   - [NASA CGI Moon Kit](https://svs.gsfc.nasa.gov/cgi-bin/details.cgi?aid=4720)

2. Save as `textures/moon.jpg`

3. The simulation will automatically load it on startup

See `textures/README.md` for more details.
