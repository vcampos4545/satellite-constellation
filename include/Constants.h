#ifndef CONSTANTS_H
#define CONSTANTS_H

// Mathematical constants
const double PI = 3.14159265359;

// Gravitational constant
const double G = 6.67430e-11; // m^3 kg^-1 s^-2

// Earth constants
const double EARTH_MASS = 5.972e24;                                          // kg
const double EARTH_RADIUS = 6.371e6;                                         // meters
const double EARTH_ROTATION_ANGULAR_VELOCITY = 360.0 / (24.0 * 60.0 * 60.0); // Radians per second
const double EARTH_AXIAL_TILT = 23.5;                                        // degrees

// Sun constants
const double SUN_MASS = 1.989e30; // kg
const double SUN_RADIUS = 6.96e8; // meters
const double AU = 1.496e11;       // Astronomical Unit in meters

// Moon constants
const double MOON_MASS = 7.342e22;                                             // kg
const double MOON_RADIUS = 1.7371e6;                                           // meters
const double MOON_SEMI_MAJOR_AXIS = 3.844e8;                                   // meters (384,400 km from Earth)
const double MOON_ECCENTRICITY = 0.0549;                                       // Eccentricity (slightly elliptical)
const double MOON_INCLINATION_TO_ECLIPTIC = 5.145;                             // degrees (to ecliptic plane)
const double MOON_ORBITAL_PERIOD = 27.3 * 24.0 * 3600.0;                       // seconds (sidereal month)
const double MOON_ANGULAR_VELOCITY = 2.0 * PI / MOON_ORBITAL_PERIOD;           // radians per second (orbital)
const double MOON_ROTATION_PERIOD = MOON_ORBITAL_PERIOD;                       // Tidally locked (same as orbital period)
const double MOON_ROTATION_ANGULAR_VELOCITY = 2.0 * PI / MOON_ROTATION_PERIOD; // radians per second
const double MOON_AXIAL_TILT = 6.68;                                           // degrees (obliquity to its orbital plane)

// Ecliptic plane angle (angle between Earth's equator and ecliptic)
const double ECLIPTIC_OBLIQUITY = 23.44; // degrees (Earth's axial tilt)

// Satellite orbit altitudes
const double GEO_ALTITUDE = 35.786e6; // GEO altitude above Earth surface (meters)
const double LEO_ALTITUDE = 550e3;    // Starlink altitude ~550 km
const double GPS_ALTITUDE = 20.2e6;   // Altitude of GPS satellites

// Molniya orbit parameters (highly elliptical orbit for high latitude coverage)
const double MOLNIYA_SEMI_MAJOR_AXIS = 26.6e6; // Semi-major axis (meters from Earth center)
const double MOLNIYA_ECCENTRICITY = 0.72;      // Eccentricity (highly elliptical)
const double MOLNIYA_INCLINATION = 63.4;       // Inclination (degrees) - critical angle
const double MOLNIYA_APOGEE_ALTITUDE = 39.8e6; // Apogee altitude above surface (~40,000 km)
const double MOLNIYA_PERIGEE_ALTITUDE = 500e3; // Perigee altitude above surface (~500 km)

// Atmospheric constants (exponential atmosphere model)
const double RHO_0 = 1.225;              // Sea level atmospheric density (kg/m^3)
const double H_SCALE = 8500.0;           // Scale height for exponential atmosphere (meters)
const double DRAG_ALTITUDE_MAX = 1000e3; // Maximum altitude where drag is significant (1000 km)

// Solar radiation pressure constants
const double SOLAR_FLUX = 1367.0;                          // Solar constant at 1 AU (W/m^2)
const double SPEED_OF_LIGHT = 299792458.0;                 // Speed of light (m/s)
const double SOLAR_PRESSURE = SOLAR_FLUX / SPEED_OF_LIGHT; // Pressure at 1 AU (N/m^2)

#endif // CONSTANTS_H
