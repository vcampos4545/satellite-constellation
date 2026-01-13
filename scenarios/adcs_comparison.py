#!/usr/bin/env python3
"""
Example: ADCS Algorithm Comparison

This scenario demonstrates:
- Comparing different flight software algorithms side-by-side
- Testing multiple satellites with identical orbits but different ADCS
- Analyzing performance of different control strategies
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))

import satellite_sim as sim


def run(show_gui=True):
    """
    Test multiple ADCS algorithms on identical satellites.

    Args:
        show_gui (bool): Whether to display the GUI
    """

    print("=" * 60)
    print("ADCS ALGORITHM COMPARISON")
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

    # Add ground stations for ground-tracking ADCS
    print("[3/5] Adding ground stations...")
    cities = [
        ("Houston", 29.7604, -95.3698),
        ("Cape Canaveral", 28.3922, -80.6077),
        ("Vandenberg", 34.7420, -120.5724),
    ]
    for name, lat, lon in cities:
        universe.add_ground_station(name, lat, lon)
    print(f"  ✓ {len(cities)} ground stations added")

    # Create three identical satellites with different FSW
    print("[4/5] Creating test satellites with different ADCS...")

    adcs_configs = [
        {
            "name": "TestSat-Standard",
            "fsw": sim.create_standard_fsw(),
            "description": "Standard FSW (ADCS + power + station keeping)",
            "altitude_offset": 0,
            "true_anomaly": 0,
        },
        {
            "name": "TestSat-GroundTracking",
            "fsw": sim.create_ground_tracking_adcs(),
            "description": "Ground Tracking ADCS (points at nearest ground station)",
            "altitude_offset": 50,
            "true_anomaly": 120,
        },
        {
            "name": "TestSat-Passive",
            "fsw": sim.create_passive_fsw(),
            "description": "Passive FSW (minimal control)",
            "altitude_offset": 100,
            "true_anomaly": 240,
        },
    ]

    for idx, config in enumerate(adcs_configs):
        # Create orbit with slight variations to keep satellites separated
        orbit = sim.Orbit.from_altitude_inclination(
            altitude_km=700.0 + config["altitude_offset"],
            inclination_deg=45.0,
            eccentricity=0.0,
            true_anomaly_deg=config["true_anomaly"]
        )

        # Add satellite
        universe.add_satellite_with_orbit(
            orbit=orbit,
            plane_id=idx,
            index_in_plane=0,
            name=config["name"],
            type=sim.SatelliteType.CUBESAT_1U
        )

        # Assign flight software
        satellites = universe.get_satellites()
        satellites[-1].set_flight_software(config["fsw"])

        print(f"  ✓ {config['name']}")
        print(f"    {config['description']}")
        print(f"    Altitude: {700 + config['altitude_offset']} km, "
              f"True anomaly: {config['true_anomaly']}°")

    print(f"\n  Total satellites: {len(universe.get_satellites())}")

    # Set simulation parameters
    simulation.set_time_warp(100.0)

    # Run simulation
    print("[5/5] Starting simulation...")
    print(f"  Time warp: {simulation.get_time_warp():.0f}x")
    print(f"  GUI: {'Enabled' if show_gui else 'Disabled'}")
    print("\n" + "=" * 60)
    print("COMPARE THE THREE SATELLITES:")
    print("  - Standard FSW: General-purpose control")
    print("  - Ground Tracking: Always points at nearest station")
    print("  - Passive: Minimal control (tumbling)")
    print("=" * 60 + "\n")

    simulation.run()

    print("\n✓ Simulation completed successfully!")
    return simulation


if __name__ == "__main__":
    show_gui = "--headless" not in sys.argv
    run(show_gui=show_gui)
