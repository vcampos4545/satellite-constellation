# Satellite Constellation Simulation Scenarios

This directory contains example Python scripts that demonstrate how to configure and run satellite constellation simulations using the `satellite_sim` Python module.

## Getting Started

### Prerequisites

1. **Build the project with Python bindings:**
   ```bash
   cd /path/to/satelitte-constelation
   mkdir build && cd build
   cmake ..
   make
   make install  # Installs satellite_sim.so to scenarios/ directory
   ```

2. **Ensure pybind11 is installed:**
   ```bash
   # Using pip
   pip install pybind11

   # Or using conda
   conda install -c conda-forge pybind11
   ```

3. **Python 3.7+ is required**

### Running Example Scenarios

Each scenario is a standalone Python script. Run them directly:

```bash
# Single CubeSat with ground tracking
python3 scenarios/example_cubesat.py

# Starlink-like constellation
python3 scenarios/starlink_constellation.py

# Compare different ADCS algorithms
python3 scenarios/adcs_comparison.py

# GPS constellation
python3 scenarios/gps_constellation.py

# Run without GUI (headless mode)
python3 scenarios/example_cubesat.py --headless
```

## Example Scenarios

### 1. `example_cubesat.py` - Simple Single Satellite
- **Purpose:** Introduction to basic simulation setup
- **Features:**
  - Single 1U CubeSat at 700 km altitude
  - Ground tracking ADCS (points at nearest ground station)
  - Major city ground stations
  - 100x time warp
- **Good for:** Learning the API, testing FSW algorithms

### 2. `starlink_constellation.py` - Large LEO Constellation
- **Purpose:** Demonstrate constellation building
- **Features:**
  - Starlink-like constellation (default: 8 planes × 4 sats = 32 total)
  - 550 km altitude, 53° inclination
  - Customizable via command-line arguments
  - Programmatic orbit generation
- **Usage:**
  ```bash
  python3 starlink_constellation.py --planes 6 --sats-per-plane 10
  ```
- **Good for:** Coverage analysis, constellation design

### 3. `adcs_comparison.py` - Flight Software Testing
- **Purpose:** Compare different ADCS algorithms
- **Features:**
  - Three identical satellites with different FSW:
    - Standard FSW (full ADCS + power + station keeping)
    - Ground Tracking ADCS (nadir pointing)
    - Passive FSW (minimal control)
  - Side-by-side comparison
- **Good for:** FSW development, algorithm validation

### 4. `gps_constellation.py` - GPS System
- **Purpose:** Medium Earth Orbit (MEO) constellation
- **Features:**
  - Full 24-satellite GPS constellation
  - 6 orbital planes, 55° inclination
  - ~20,200 km altitude
  - 1000x time warp (slower orbits)
- **Good for:** Global navigation system study

## Creating Your Own Scenarios

### Basic Template

```python
#!/usr/bin/env python3
import sys
import os

# Add build directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))
import satellite_sim as sim

def run(show_gui=True):
    # 1. Create simulation
    simulation = sim.Simulation(headless=not show_gui)
    universe = simulation.get_universe()

    # 2. Initialize celestial bodies
    universe.initialize_earth()
    universe.initialize_sun()
    universe.initialize_moon()  # Optional

    # 3. Add satellites
    orbit = sim.Orbit.from_altitude_inclination(
        altitude_km=700,
        inclination_deg=45
    )
    universe.add_satellite_with_orbit(
        orbit=orbit,
        name="MySatellite",
        type=sim.SatelliteType.CUBESAT_1U
    )

    # 4. Configure flight software
    satellites = universe.get_satellites()
    satellites[0].set_flight_software(sim.create_standard_fsw())

    # 5. Set time warp and run
    simulation.set_time_warp(100.0)
    simulation.run()

if __name__ == "__main__":
    run()
```

## API Reference

### Core Classes

#### `Simulation`
```python
sim = Simulation(headless=False)
sim.run()                    # Start simulation (blocking)
sim.set_time_warp(100.0)     # Set time speed multiplier
sim.get_universe()           # Access universe object
```

#### `Universe`
```python
universe = sim.Universe()
universe.initialize_earth()
universe.initialize_sun()
universe.initialize_moon()
universe.add_satellite_with_orbit(orbit, name="Sat1", type=sim.SatelliteType.DEFAULT)
universe.add_ground_station("Houston", 29.76, -95.37)
universe.get_satellites()      # Returns list of satellites
universe.get_ground_stations() # Returns list of ground stations
```

#### `Orbit`
```python
# Method 1: From altitude and inclination (easiest)
orbit = sim.Orbit.from_altitude_inclination(
    altitude_km=700,
    inclination_deg=45,
    eccentricity=0.0,          # Optional
    raan_deg=0,                # Optional
    arg_perigee_deg=0,         # Optional
    true_anomaly_deg=0         # Optional
)

# Method 2: Direct orbital elements
orbit = sim.Orbit()
orbit.a = 7071000.0            # Semi-major axis (meters)
orbit.e = 0.0                  # Eccentricity
orbit.i = sim.deg_to_rad(45)   # Inclination (radians)
orbit.omega = 0.0              # RAAN (radians)
orbit.w = 0.0                  # Argument of perigee (radians)
orbit.v = 0.0                  # True anomaly (radians)
```

#### `Satellite`
```python
sat = satellites[0]
sat.set_flight_software(fsw)
sat.enable_reaction_wheels(True)
sat.enable_magnetorquers(True)
sat.set_mass(5.0)              # kg
sat.get_position()             # Returns (x, y, z) tuple
sat.get_battery_percentage()
```

### Satellite Types
```python
sim.SatelliteType.DEFAULT      # Generic satellite
sim.SatelliteType.CUBESAT_1U   # 1U CubeSat (10x10x10 cm)
sim.SatelliteType.CUBESAT_2U   # 2U CubeSat (10x10x20 cm)
sim.SatelliteType.STARLINK     # Starlink-like satellite
```

### Flight Software
```python
# Available FSW modules
sim.create_standard_fsw()           # Full ADCS + power + station keeping
sim.create_ground_tracking_adcs()   # Points at nearest ground station
sim.create_passive_fsw()            # Minimal control
```

### Constellation Builders
```python
# Preset constellations
sim.add_gps_constellation(universe)
sim.add_starlink_constellation(universe, num_planes=8, sats_per_plane=4)
sim.add_geo_constellation(universe, num_satellites=3)
sim.add_molniya_constellation(universe, num_satellites=3)
sim.add_cities(universe)  # Add ground stations at major cities
```

### Utility Functions
```python
sim.deg_to_rad(45)         # Convert degrees to radians
sim.rad_to_deg(0.785)      # Convert radians to degrees
sim.km_to_m(700)           # Convert km to meters
sim.m_to_km(700000)        # Convert meters to km
```

## Advanced Usage

### Custom Constellation Patterns

```python
# Walker Delta constellation
num_planes = 6
sats_per_plane = 11
altitude_km = 1200
inclination_deg = 87

for plane in range(num_planes):
    raan_deg = plane * (360.0 / num_planes)

    for sat in range(sats_per_plane):
        true_anomaly_deg = sat * (360.0 / sats_per_plane)

        orbit = sim.Orbit.from_altitude_inclination(
            altitude_km=altitude_km,
            inclination_deg=inclination_deg,
            raan_deg=raan_deg,
            true_anomaly_deg=true_anomaly_deg
        )

        universe.add_satellite_with_orbit(
            orbit=orbit,
            plane_id=plane,
            index_in_plane=sat,
            name=f"Sat-{plane}-{sat}",
            type=sim.SatelliteType.DEFAULT
        )
```

### Custom Satellite Configuration

```python
# Add satellite
universe.add_satellite_with_orbit(orbit, name="CustomSat")
sat = universe.get_satellites()[-1]

# Configure hardware
sat.enable_reaction_wheels(True)
sat.enable_magnetorquers(True)
sat.set_mass(10.0)                    # kg
sat.set_drag_coefficient(2.2)
sat.set_cross_sectional_area(0.1)     # m²
sat.set_reflectivity(1.3)

# Assign flight software
sat.set_flight_software(sim.create_standard_fsw())
```

## Tips

1. **Time Warp:** Adjust based on orbit altitude
   - LEO (500-700 km): 100-500x
   - MEO (20,000 km): 1000-5000x
   - GEO (35,786 km): 5000-10000x

2. **Headless Mode:** Use for batch simulations or CI/CD
   ```python
   simulation = sim.Simulation(headless=True)
   ```

3. **Ground Stations:** Add custom locations
   ```python
   universe.add_ground_station("My Station", latitude_deg=40.0, longitude_deg=-75.0)
   ```

4. **Satellite Names:** Use descriptive names for easier debugging
   ```python
   name=f"Shell1-Plane{plane}-Sat{sat}"
   ```

## Troubleshooting

### Module Not Found
```
ImportError: No module named 'satellite_sim'
```
**Solution:** Ensure you've built and installed the Python module:
```bash
cd build
make
make install
```

### Simulation Crashes
- Check that celestial bodies are initialized before adding satellites
- Ensure orbit parameters are physically valid (e.g., altitude > 200 km)
- Verify time warp isn't too high (causing numerical instability)

### Empty Visualization
- Make sure you called `universe.initialize_earth()` before running
- Verify satellites were added with valid orbits

## Contributing

To add new example scenarios:
1. Create a new `.py` file in this directory
2. Follow the template structure above
3. Add documentation at the top explaining the scenario
4. Update this README with a description

## Support

For issues or questions:
- Check the main project README
- Review existing scenarios for examples
- Open an issue on the project repository
