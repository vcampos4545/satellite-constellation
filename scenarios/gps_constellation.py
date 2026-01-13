#!/usr/bin/env python3
"""
Example: GPS Constellation

This scenario demonstrates:
- Using preset constellation builders
- GPS constellation (24 satellites, 6 planes, MEO altitude)
- Global navigation satellite system architecture
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))

import satellite_sim as sim


def run(show_gui=True):
    """
    Create a full GPS constellation.

    Args:
        show_gui (bool): Whether to display the GUI
    """

    print("=" * 60)
    print("GPS CONSTELLATION (24 SATELLITES)")
    print("=" * 60)

    # Create simulation
    print("\n[1/4] Creating simulation...")
    simulation = sim.Simulation(headless=not show_gui)
    universe = simulation.get_universe()

    # Initialize celestial bodies
    print("[2/4] Initializing celestial bodies...")
    universe.initialize_earth()
    universe.initialize_sun()
    universe.initialize_moon()
    print("  ✓ Earth, Sun, and Moon initialized")

    # Add GPS constellation
    print("[3/4] Creating GPS constellation...")
    print("  Configuration:")
    print("    - 24 satellites total")
    print("    - 6 orbital planes")
    print("    - 4 satellites per plane")
    print("    - ~20,200 km altitude (MEO)")
    print("    - 55° inclination")

    sim.add_gps_constellation(universe)

    satellites = universe.get_satellites()
    print(f"  ✓ {len(satellites)} GPS satellites created")

    # Optionally add ground stations to visualize coverage
    print("\n  Adding ground stations for coverage visualization...")
    sim.add_cities(universe)
    print(f"  ✓ {len(universe.get_ground_stations())} ground stations added")

    # Set simulation parameters
    # GPS satellites orbit slower (12 hour period), so use higher time warp
    simulation.set_time_warp(1000.0)

    # Run simulation
    print("[4/4] Starting simulation...")
    print(f"  Time warp: {simulation.get_time_warp():.0f}x")
    print(f"  GPS orbital period: ~12 hours")
    print(f"  GUI: {'Enabled' if show_gui else 'Disabled'}")
    print("\n" + "=" * 60)
    print("GPS CONSTELLATION INFO:")
    print("  - Medium Earth Orbit (MEO)")
    print("  - Global coverage with 24 satellites")
    print("  - Each satellite visible for ~5 hours")
    print("  - 4-6 satellites visible at any location")
    print("=" * 60 + "\n")

    simulation.run()

    print("\n✓ Simulation completed successfully!")
    return simulation


if __name__ == "__main__":
    show_gui = "--headless" not in sys.argv
    run(show_gui=show_gui)
