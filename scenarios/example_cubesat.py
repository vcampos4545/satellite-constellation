#!/usr/bin/env python3
"""
Example: Single CubeSat with Ground Tracking ADCS

This scenario demonstrates:
- Creating a simple simulation with one CubeSat
- Setting up ground stations at major cities
- Configuring ground-tracking ADCS
- Running with GUI visualization
"""

import sys
import os

# Add parent directory to path to find satellite_sim module
# (This assumes the module is built in the build directory)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))

import satellite_sim as sim


def run(show_gui=True):
    """
    Configure and run a single CubeSat simulation.

    Args:
        show_gui (bool): Whether to display the GUI (default: True)
    """

    print("=" * 60)
    print("SINGLE CUBESAT WITH GROUND TRACKING ADCS")
    print("=" * 60)

    # Create simulation
    print("\n[1/6] Creating simulation...")
    simulation = sim.Simulation(headless=not show_gui)
    universe = simulation.get_universe()

    # Initialize celestial bodies
    print("[2/6] Initializing celestial bodies...")
    universe.initialize_earth()
    universe.initialize_sun()
    universe.initialize_moon()
    print("  ✓ Earth, Sun, and Moon initialized")

    # Add ground stations at major cities
    print("[3/6] Adding ground stations...")
    sim.add_cities(universe)
    print(f"  ✓ {len(universe.get_ground_stations())} ground stations added")

    # Create orbit for CubeSat (700 km altitude, 45° inclination)
    print("[4/6] Creating CubeSat orbit...")
    orbit = sim.Orbit.from_altitude_inclination(
        altitude_km=700.0,
        inclination_deg=45.0,
        eccentricity=0.0
    )
    print(f"  ✓ Orbit: {orbit.get_elements_deg()['altitude_km']:.0f} km altitude, "
          f"{orbit.get_elements_deg()['inclination_deg']:.1f}° inclination")

    # Add CubeSat
    print("[5/6] Adding CubeSat with GroundTrackingADCS...")
    universe.add_satellite_with_orbit(
        orbit=orbit,
        plane_id=0,
        index_in_plane=0,
        name="GroundTracker-1",
        type=sim.SatelliteType.CUBESAT_1U
    )

    # Assign flight software
    satellites = universe.get_satellites()
    satellites[0].set_flight_software(sim.create_ground_tracking_adcs())
    print(f"  ✓ CubeSat '{satellites[0].get_name()}' created")

    # Set simulation time warp
    simulation.set_time_warp(100.0)  # 100x speed

    # Run simulation
    print("[6/6] Starting simulation...")
    print(f"  Time warp: {simulation.get_time_warp():.0f}x")
    print(f"  GUI: {'Enabled' if show_gui else 'Disabled'}")
    print("\n" + "=" * 60)
    print("Simulation running... Close window to exit.")
    print("=" * 60 + "\n")

    simulation.run()

    print("\n✓ Simulation completed successfully!")
    return simulation


if __name__ == "__main__":
    # Parse command line arguments
    show_gui = "--headless" not in sys.argv
    run(show_gui=show_gui)
