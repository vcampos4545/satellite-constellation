#ifndef CONSTANTS_H
#define CONSTANTS_H

// Mathematical constants
const double PI = 3.14159265359;

// Gravitational constant
const double G = 6.67430e-11; // m^3 kg^-1 s^-2

// Earth constants
const double EARTH_MASS = 5.972e24;   // kg
const double EARTH_RADIUS = 6.371e6;  // meters
const double EARTH_ROTATION_SPEED = 360.0 / (24.0 * 60.0 * 60.0); // Radians per second
const double EARTH_AXIAL_TILT = 23.5; // degrees

// Sun constants
const double SUN_MASS = 1.989e30;  // kg
const double SUN_RADIUS = 6.96e8;  // meters
const double AU = 1.496e11;        // Astronomical Unit in meters

// Moon constants
const double MOON_MASS = 7.342e22;                                 // kg
const double MOON_RADIUS = 1.7371e6;                               // meters
const double MOON_ORBIT_RADIUS = 3.844e8;                          // meters (384,400 km from Earth)
const double MOON_ORBITAL_PERIOD = 27.3 * 24.0 * 3600.0;           // seconds
const double MOON_ANGULAR_VELOCITY = 2.0 * PI / MOON_ORBITAL_PERIOD; // radians per second

// Satellite orbit altitudes
const double GEO_ALTITUDE = 35.786e6; // GEO altitude above Earth surface (meters)
const double LEO_ALTITUDE = 550e3;    // Starlink altitude ~550 km

#endif // CONSTANTS_H
