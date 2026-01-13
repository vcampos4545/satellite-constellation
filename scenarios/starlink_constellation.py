#!/usr/bin/env python3
"""
Example: Starlink-like LEO Constellation

This scenario demonstrates:
- Creating a large constellation programmatically
- Using the built-in Starlink constellation builder
- Customizing satellite configurations
- Walker Delta constellation patterns
"""

import sys
import os
import math

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))

import satellite_sim as sim


def run(show_gui=True, num_planes=8, sats_per_plane=4):
    """
    Create a Starlink-like constellation.

    Args:
        show_gui (bool): Whether to display the GUI
        num_planes (int): Number of orbital planes
        sats_per_plane (int): Number of satellites per plane
    """

    print("=" * 60)
    print("STARLINK-LIKE CONSTELLATION")
    print("=" * 60)

    # Create simulation
    print("\n[1/5] Creating simulation...")
    simulation = sim.Simulation(headless=not show_gui)
    universe = simulation.get_universe()

    # Initialize celestial bodies
    print("[2/5] Initializing celestial bodies...")
    universe.initialize_earth()
    universe.initialize_sun()
    print("  ✓ Earth and Sun initialized")

    # Use built-in Starlink constellation builder
    print(f"[3/5] Creating Starlink constellation...")
    print(f"  Configuration: {num_planes} planes × {sats_per_plane} satellites/plane")
    sim.add_starlink_constellation(universe, num_planes, sats_per_plane)

    total_sats = num_planes * sats_per_plane
    print(f"  ✓ {total_sats} satellites created at 550 km altitude, 53° inclination")

    # Alternatively, create custom constellation programmatically
    print("[4/5] Adding custom demonstration satellites...")
    custom_orbits = [
        {"alt": 600, "inc": 45, "name": "CustomSat-A"},
        {"alt": 650, "inc": 60, "name": "CustomSat-B"},
        {"alt": 700, "inc": 75, "name": "CustomSat-C"},
    ]

    for i, cfg in enumerate(custom_orbits):
        orbit = sim.Orbit.from_altitude_inclination(
            altitude_km=cfg["alt"],
            inclination_deg=cfg["inc"],
            eccentricity=0.0,
            true_anomaly_deg=i * 120  # Spread around orbit
        )
        universe.add_satellite_with_orbit(
            orbit=orbit,
            plane_id=100 + i,  # Use high plane IDs to distinguish from Starlink
            index_in_plane=0,
            name=cfg["name"],
            type=sim.SatelliteType.DEFAULT
        )

    total_satellites = len(universe.get_satellites())
    print(f"  ✓ Total satellites: {total_satellites}")

    # Set simulation parameters
    simulation.set_time_warp(500.0)  # Fast time warp for large constellations

    # Run simulation
    print("[5/5] Starting simulation...")
    print(f"  Time warp: {simulation.get_time_warp():.0f}x (faster for large constellations)")
    print(f"  GUI: {'Enabled' if show_gui else 'Disabled'}")
    print("\n" + "=" * 60)
    print("Simulation running... Close window to exit.")
    print("=" * 60 + "\n")

    simulation.run()

    print("\n✓ Simulation completed successfully!")
    return simulation


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Starlink constellation simulation")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI")
    parser.add_argument("--planes", type=int, default=8,
                        help="Number of orbital planes (default: 8)")
    parser.add_argument("--sats-per-plane", type=int, default=4,
                        help="Satellites per plane (default: 4)")

    args = parser.parse_args()

    run(show_gui=not args.headless,
        num_planes=args.planes,
        sats_per_plane=args.sats_per_plane)
