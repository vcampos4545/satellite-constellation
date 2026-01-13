/**
 * Python Bindings for Satellite Constellation Simulator
 *
 * This file uses pybind11 to expose C++ classes to Python,
 * enabling Basilisk-style scenario scripting where users can
 * programmatically configure simulations in Python.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "Simulation.h"
#include "Universe.h"
#include "Satellite.h"
#include "Orbit.h"
#include "CelestialBody.h"
#include "GroundStation.h"
#include "Constants.h"
#include "FlightSoftwareTask.h"
#include "StandardFSW.h"
#include "PassiveFSW.h"
#include "GroundTrackingADCS.h"
#include <glm/glm.hpp>
#include <cmath>

namespace py = pybind11;

// Helper function to create orbit from altitude and inclination
Orbit createOrbitFromAltitudeInclination(double altitude_km, double inclination_deg,
                                          double eccentricity = 0.0,
                                          double raan_deg = 0.0,
                                          double arg_perigee_deg = 0.0,
                                          double true_anomaly_deg = 0.0)
{
    Orbit orbit;
    orbit.a = EARTH_RADIUS + (altitude_km * 1000.0);
    orbit.e = eccentricity;
    orbit.i = inclination_deg * M_PI / 180.0;
    orbit.omega = raan_deg * M_PI / 180.0;
    orbit.w = arg_perigee_deg * M_PI / 180.0;
    orbit.v = true_anomaly_deg * M_PI / 180.0;
    return orbit;
}

// Helper to get orbit elements in degrees (more user-friendly)
py::dict getOrbitElementsDeg(const Orbit& orbit)
{
    py::dict d;
    d["semi_major_axis_km"] = orbit.a / 1000.0;
    d["altitude_km"] = (orbit.a - EARTH_RADIUS) / 1000.0;
    d["eccentricity"] = orbit.e;
    d["inclination_deg"] = orbit.i * 180.0 / M_PI;
    d["raan_deg"] = orbit.omega * 180.0 / M_PI;
    d["arg_perigee_deg"] = orbit.w * 180.0 / M_PI;
    d["true_anomaly_deg"] = orbit.v * 180.0 / M_PI;
    return d;
}

PYBIND11_MODULE(satellite_sim, m) {
    m.doc() = "Satellite Constellation Simulator - Python Interface";

    // ==================== CONSTANTS ====================

    py::class_<py::object>(m, "Constants")
        .def_readonly_static("PI", &PI)
        .def_readonly_static("G", &G)
        .def_readonly_static("EARTH_MASS", &EARTH_MASS)
        .def_readonly_static("EARTH_RADIUS", &EARTH_RADIUS)
        .def_readonly_static("EARTH_ROTATION_RATE", &EARTH_ROTATION_ANGULAR_VELOCITY)
        .def_readonly_static("SUN_MASS", &SUN_MASS)
        .def_readonly_static("SUN_RADIUS", &SUN_RADIUS)
        .def_readonly_static("MOON_MASS", &MOON_MASS)
        .def_readonly_static("MOON_RADIUS", &MOON_RADIUS)
        .def_readonly_static("AU", &AU);

    // ==================== ORBIT ====================

    py::class_<Orbit>(m, "Orbit")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("semi_major_axis"),
             py::arg("eccentricity"),
             py::arg("inclination_rad"),
             py::arg("raan_rad"),
             py::arg("arg_perigee_rad"),
             py::arg("true_anomaly_rad"))
        .def_readwrite("a", &Orbit::a, "Semi-major axis (meters)")
        .def_readwrite("e", &Orbit::e, "Eccentricity (0-1)")
        .def_readwrite("i", &Orbit::i, "Inclination (radians)")
        .def_readwrite("omega", &Orbit::omega, "RAAN - Right Ascension of Ascending Node (radians)")
        .def_readwrite("w", &Orbit::w, "Argument of perigee (radians)")
        .def_readwrite("v", &Orbit::v, "True anomaly (radians)")
        .def_static("from_altitude_inclination", &createOrbitFromAltitudeInclination,
             py::arg("altitude_km"),
             py::arg("inclination_deg"),
             py::arg("eccentricity") = 0.0,
             py::arg("raan_deg") = 0.0,
             py::arg("arg_perigee_deg") = 0.0,
             py::arg("true_anomaly_deg") = 0.0,
             "Create orbit from altitude (km) and inclination (degrees)")
        .def("get_elements_deg", &getOrbitElementsDeg,
             "Get orbital elements as dictionary with degrees and km")
        .def("to_cartesian", &Orbit::toCartesian,
             py::arg("position"),
             py::arg("velocity"),
             py::arg("mu"),
             "Convert orbital elements to Cartesian state");

    // ==================== SATELLITE TYPE ====================

    py::enum_<SatelliteType>(m, "SatelliteType")
        .value("DEFAULT", SatelliteType::DEFAULT, "Generic satellite")
        .value("CUBESAT_1U", SatelliteType::CUBESAT_1U, "1U CubeSat (10x10x10 cm)")
        .value("CUBESAT_2U", SatelliteType::CUBESAT_2U, "2U CubeSat (10x10x20 cm)")
        .value("STARLINK", SatelliteType::STARLINK, "Starlink satellite")
        .export_values();

    // ==================== SATELLITE ====================

    py::class_<Satellite, std::shared_ptr<Satellite>>(m, "Satellite")
        .def("set_flight_software", &Satellite::setFlightSoftware,
             py::arg("fsw"),
             "Set the flight software controller for this satellite")
        .def("get_flight_software", &Satellite::getFlightSoftware,
             "Get current flight software")
        .def("get_position", [](const Satellite &sat) {
            auto pos = sat.getPosition();
            return py::make_tuple(pos.x, pos.y, pos.z);
        }, "Get position (x, y, z) in meters")
        .def("get_velocity", [](const Satellite &sat) {
            auto vel = sat.getVelocity();
            return py::make_tuple(vel.x, vel.y, vel.z);
        }, "Get velocity (vx, vy, vz) in m/s")
        .def("get_name", &Satellite::getName, "Get satellite name")
        .def("get_type", &Satellite::getType, "Get satellite type")
        .def("get_battery_percentage", &Satellite::getBatteryPercentage,
             "Get battery charge percentage")
        .def("get_propellant_fraction", &Satellite::getPropellantFraction,
             "Get remaining propellant fraction (0-1)")
        .def("is_in_eclipse", &Satellite::isInEclipse,
             "Check if satellite is in Earth's shadow")
        // Actuator configuration
        .def("enable_reaction_wheels", &Satellite::enableReactionWheels,
             py::arg("enable"),
             "Enable/disable reaction wheels")
        .def("enable_magnetorquers", &Satellite::enableMagnetorquers,
             py::arg("enable"),
             "Enable/disable magnetorquers")
        .def("enable_cmgs", &Satellite::enableCMGs,
             py::arg("enable"),
             "Enable/disable Control Moment Gyroscopes")
        // Physical property setters
        .def("set_mass", &Satellite::setMass, py::arg("mass_kg"),
             "Set satellite mass in kg")
        .def("set_drag_coefficient", &Satellite::setDragCoefficient,
             py::arg("cd"),
             "Set drag coefficient")
        .def("set_cross_sectional_area", &Satellite::setCrossSectionalArea,
             py::arg("area_m2"),
             "Set cross-sectional area in m²")
        .def("set_reflectivity", &Satellite::setReflectivity,
             py::arg("cr"),
             "Set solar radiation pressure reflectivity coefficient");

    // ==================== CELESTIAL BODY ====================

    py::class_<CelestialBody, std::shared_ptr<CelestialBody>>(m, "CelestialBody")
        .def("get_position", [](const CelestialBody &body) {
            auto pos = body.getPosition();
            return py::make_tuple(pos.x, pos.y, pos.z);
        }, "Get position (x, y, z) in meters")
        .def("get_mass", &CelestialBody::getMass, "Get mass in kg")
        .def("get_radius", &CelestialBody::getRadius, "Get radius in meters");

    // ==================== GROUND STATION ====================

    py::class_<GroundStation, std::shared_ptr<GroundStation>>(m, "GroundStation")
        .def("get_name", &GroundStation::getName, "Get ground station name")
        .def("get_latitude", &GroundStation::getLatitude, "Get latitude in degrees")
        .def("get_longitude", &GroundStation::getLongitude, "Get longitude in degrees")
        .def("get_position", [](const GroundStation &gs) {
            auto pos = gs.getPosition();
            return py::make_tuple(pos.x, pos.y, pos.z);
        }, "Get ECEF position (x, y, z) in meters");

    // ==================== FLIGHT SOFTWARE ====================

    py::class_<FlightSoftwareTask, std::shared_ptr<FlightSoftwareTask>>(m, "FlightSoftwareTask")
        .def("get_name", &FlightSoftwareTask::getName,
             "Get the name of this flight software task")
        .def("reset", &FlightSoftwareTask::reset,
             "Reset flight software state");

    // Flight software factory functions
    m.def("create_standard_fsw", []() {
        return std::static_pointer_cast<FlightSoftwareTask>(
            std::make_shared<StandardFSW>());
    }, "Create standard flight software (ADCS + power + station keeping)");

    m.def("create_passive_fsw", []() {
        return std::static_pointer_cast<FlightSoftwareTask>(
            std::make_shared<PassiveFSW>());
    }, "Create passive flight software (minimal control)");

    m.def("create_ground_tracking_adcs", []() {
        return std::static_pointer_cast<FlightSoftwareTask>(
            std::make_shared<GroundTrackingADCS>());
    }, "Create ground tracking ADCS (points at nearest ground station)");

    // ==================== UNIVERSE ====================

    py::class_<Universe>(m, "Universe")
        .def(py::init<>(), "Create a new empty universe")

        // Celestial body initialization
        .def("initialize_earth", &Universe::initializeEarth,
             "Initialize Earth at the origin")
        .def("initialize_sun", &Universe::initializeSun,
             "Initialize Sun with realistic orbital motion")
        .def("initialize_moon", &Universe::initializeMoon,
             "Initialize Moon with realistic orbital motion")

        // Access celestial bodies
        .def("get_earth", &Universe::getEarth,
             "Get Earth celestial body")
        .def("get_sun", &Universe::getSun,
             "Get Sun celestial body")
        .def("get_moon", &Universe::getMoon,
             "Get Moon celestial body")

        // Satellite management
        .def("add_satellite_with_orbit", &Universe::addSatelliteWithOrbit,
             py::arg("orbit"),
             py::arg("plane_id") = 0,
             py::arg("index_in_plane") = 0,
             py::arg("name") = "",
             py::arg("type") = SatelliteType::DEFAULT,
             "Add a satellite with specified orbital elements")
        .def("get_satellites",
             [](Universe &u) { return u.getSatellites(); },
             py::return_value_policy::reference,
             "Get list of all satellites")

        // Ground station management
        .def("add_ground_station", &Universe::addGroundStation,
             py::arg("name"),
             py::arg("latitude_deg"),
             py::arg("longitude_deg"),
             "Add a ground station at specified lat/lon")
        .def("get_ground_stations",
             [](Universe &u) { return u.getGroundStations(); },
             py::return_value_policy::reference,
             "Get list of all ground stations")

        // Simulation control
        .def("update", &Universe::update,
             py::arg("delta_time"),
             py::arg("max_physics_step") = 0.1,
             "Update universe physics for given time step")
        .def("get_simulation_time", &Universe::getSimulationTime,
             "Get total elapsed simulation time in seconds");

    // Constellation builder helper functions
    m.def("add_gps_constellation", [](Universe* universe) {
        // Call the C++ helper function
        extern void addGPSConstellation(Universe*);
        addGPSConstellation(universe);
    }, py::arg("universe"),
    "Add full GPS constellation (24 satellites, 6 planes, 55° inclination)");

    m.def("add_geo_constellation", [](Universe* universe, int num_satellites) {
        extern void addGEOConstellation(Universe*, int);
        addGEOConstellation(universe, num_satellites);
    }, py::arg("universe"), py::arg("num_satellites") = 3,
    "Add geostationary constellation");

    m.def("add_starlink_constellation", [](Universe* universe, int num_planes, int sats_per_plane) {
        extern void addStarlinkConstellation(Universe*, int, int);
        addStarlinkConstellation(universe, num_planes, sats_per_plane);
    }, py::arg("universe"),
       py::arg("num_planes") = 8,
       py::arg("sats_per_plane") = 4,
    "Add Starlink-like constellation (default: 8 planes, 4 sats/plane, 550 km altitude)");

    m.def("add_molniya_constellation", [](Universe* universe, int num_satellites) {
        extern void addMolniyaConstellation(Universe*, int);
        addMolniyaConstellation(universe, num_satellites);
    }, py::arg("universe"), py::arg("num_satellites") = 3,
    "Add Molniya highly elliptical orbit constellation");

    m.def("add_cities", [](Universe* universe) {
        extern void addCities(Universe*);
        addCities(universe);
    }, py::arg("universe"),
    "Add ground stations at major world cities");

    // ==================== SIMULATION ====================

    py::class_<Simulation>(m, "Simulation")
        .def(py::init<bool>(),
             py::arg("headless") = false,
             "Create simulation (headless=True for no GUI)")
        .def("run", &Simulation::run,
             "Run the simulation (blocking until window closed)")
        .def("set_paused", &Simulation::setPaused,
             py::arg("paused"),
             "Pause/unpause the simulation")
        .def("is_paused", &Simulation::isPaused,
             "Check if simulation is paused")
        .def("set_time_warp", &Simulation::setTimeWarp,
             py::arg("multiplier"),
             "Set time warp multiplier (e.g., 100 = 100x speed)")
        .def("get_time_warp", &Simulation::getTimeWarp,
             "Get current time warp multiplier")
        .def("get_elapsed_time", &Simulation::getElapsedTime,
             "Get total elapsed simulation time in seconds")
        .def("get_universe",
             py::overload_cast<>(&Simulation::getUniverse),
             py::return_value_policy::reference,
             "Get reference to the universe");

    // ==================== UTILITY FUNCTIONS ====================

    m.def("deg_to_rad", [](double deg) { return deg * M_PI / 180.0; },
          py::arg("degrees"),
          "Convert degrees to radians");

    m.def("rad_to_deg", [](double rad) { return rad * 180.0 / M_PI; },
          py::arg("radians"),
          "Convert radians to degrees");

    m.def("km_to_m", [](double km) { return km * 1000.0; },
          py::arg("kilometers"),
          "Convert kilometers to meters");

    m.def("m_to_km", [](double m) { return m / 1000.0; },
          py::arg("meters"),
          "Convert meters to kilometers");
}
