#ifndef CONSTANTS_H
#define CONSTANTS_H

// Mathematical constants
const double PI = 3.14159265359;

// Gravitational constant
const double G = 6.67430e-11; // m^3 kg^-1 s^-2

// Earth constants
const double EARTH_MASS = 5.972e24;                                        // kg
const double EARTH_RADIUS = 6.371e6;                                       // meters (mean radius)
const double EARTH_EQUATORIAL_RADIUS = 6.378137e6;                         // meters (equatorial radius for J2 calculations)
const double EARTH_ROTATION_ANGULAR_VELOCITY = (2 * PI) / (24.0 * 3600.0); // Radians per second
const double EARTH_AXIAL_TILT = 23.5;                                      // degrees

// Earth gravity field zonal harmonic coefficients (dimensionless)
// These describe Earth's non-spherical mass distribution
const double J2 = 1.08263e-3; // Oblateness (equatorial bulge) - dominant term
const double J3 = -2.54e-6;   // Pear-shaped asymmetry (North-South)
const double J4 = -1.62e-6;   // Higher-order oblateness correction

// Solar panel thermal properties (typical values for spacecraft solar panels)
const double STEFAN_BOLTZMANN = 5.67e-8;           // W/(m²·K⁴) - Stefan-Boltzmann constant
const double SOLAR_PANEL_EMISSIVITY = 0.85;        // Back surface emissivity
const double SOLAR_PANEL_ABSORPTIVITY = 0.92;     // Solar absorptivity
const double SOLAR_PANEL_TEMP_COEFF = -0.004;     // Efficiency change per °C
const double SOLAR_PANEL_REF_TEMP = 298.15;       // Reference temp (25°C in Kelvin)
const double SOLAR_PANEL_THERMAL_MASS = 100.0;    // J/K (typical for 1kg panel)

// Sun constants
const double SUN_MASS = 1.989e30; // kg
const double SUN_RADIUS = 6.96e8; // meters
const double AU = 1.496e11;       // Astronomical Unit in meters

// Moon constants
const double MOON_MASS = 7.342e22;                           // kg
const double MOON_RADIUS = 1.7371e6;                         // meters
const double MOON_SEMI_MAJOR_AXIS = 3.844e8;                 // meters
const double MOON_ECCENTRICITY = 0.0549;                     // Eccentricity (slightly elliptical)
const double MOON_INCLINATION_TO_ECLIPTIC = 5.145;           // degrees (to ecliptic plane)
const double MOON_ROTATION_PERIOD = 27.3 * 24.0 * 3600.0;    // seconds (tidally locked ~27.3 days)
const double MOON_ROTATION_ANGULAR_VELOCITY = 2.0 * PI / MOON_ROTATION_PERIOD; // radians per second
const double MOON_AXIAL_TILT = 6.68;                         // degrees (obliquity to its orbital plane)

// Ecliptic plane angle (angle between Earth's equator and ecliptic)
const double ECLIPTIC_OBLIQUITY = 23.44; // degrees (Earth's axial tilt)

// Atmospheric constants (exponential atmosphere model)
const double RHO_0 = 1.225;              // Sea level atmospheric density (kg/m^3)
const double H_SCALE = 8500.0;           // Scale height for exponential atmosphere (meters)
const double DRAG_ALTITUDE_MAX = 1000e3; // Maximum altitude where drag is significant (1000 km)

// Solar radiation pressure constants
const double SOLAR_FLUX = 1367.0;                          // Solar constant at 1 AU (W/m^2)
const double SPEED_OF_LIGHT = 299792458.0;                 // Speed of light (m/s)
const double SOLAR_PRESSURE = SOLAR_FLUX / SPEED_OF_LIGHT; // Pressure at 1 AU (N/m^2)

static const double BOLTZMANN_CONSTANT = 1.380649e-23; // J/K

#endif // CONSTANTS_H
