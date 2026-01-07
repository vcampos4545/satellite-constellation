#ifndef CONSTANTS_H
#define CONSTANTS_H

// Mathematical constants
const double PI = 3.14159265359;

// Gravitational constant
const double G = 6.67430e-11; // m^3 kg^-1 s^-2

// Earth constants
const double EARTH_MASS = 5.972e24;                                             // kg
const double EARTH_RADIUS = 6.371e6;                                            // meters (mean radius)
const double EARTH_EQUATORIAL_RADIUS = 6.378137e6;                              // meters (equatorial radius for J2 calculations)
const double EARTH_ROTATION_ANGULAR_VELOCITY = (2 * PI) / (24.0 * 60.0 * 60.0); // Radians per second
const double EARTH_AXIAL_TILT = 23.5;                                           // degrees

// Earth gravity field zonal harmonic coefficients (dimensionless)
// These describe Earth's non-spherical mass distribution
const double J2 = 1.08263e-3; // Oblateness (equatorial bulge) - dominant term
const double J3 = -2.54e-6;   // Pear-shaped asymmetry (North-South)
const double J4 = -1.62e-6;   // Higher-order oblateness correction

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

static const double BOLTZMANN_CONSTANT = 1.380649e-23; // J/K

// Structure to hold city information
struct City
{
  const char *name;
  double latitude;  // Degrees (N positive, S negative)
  double longitude; // Degrees (E positive, W negative)
};

// Major cities around the world for ground stations
static const City MAJOR_CITIES[] = {
    // North America
    {"New York, USA", 40.7128, -74.0060},
    {"Los Angeles, USA", 34.0522, -118.2437},
    {"Chicago, USA", 41.8781, -87.6298},
    {"Houston, USA", 29.7604, -95.3698},
    {"Toronto, Canada", 43.6532, -79.3832},
    {"Mexico City, Mexico", 19.4326, -99.1332},

    // South America
    {"Sao Paulo, Brazil", -23.5505, -46.6333},
    {"Buenos Aires, Argentina", -34.6037, -58.3816},
    {"Lima, Peru", -12.0464, -77.0428},

    // // Europe
    {"London, UK", 51.5072, -0.1276},
    {"Paris, France", 48.8566, 2.3522},
    {"Berlin, Germany", 52.5200, 13.4050},
    {"Madrid, Spain", 40.4168, -3.7038},
    {"Rome, Italy", 41.9028, 12.4964},
    {"Moscow, Russia", 55.7558, 37.6173},
    {"Istanbul, Turkey", 41.0082, 28.9784},

    // Africa
    {"Cairo, Egypt", 30.0444, 31.2357},
    {"Lagos, Nigeria", 6.5244, 3.3792},
    {"Johannesburg, South Africa", -26.2041, 28.0473},
    {"Nairobi, Kenya", -1.2864, 36.8172},

    // // Asia
    {"Tokyo, Japan", 35.6762, 139.6503},
    {"Beijing, China", 39.9042, 116.4074},
    {"Shanghai, China", 31.2304, 121.4737},
    {"Mumbai, India", 19.0760, 72.8777},
    {"Delhi, India", 28.7041, 77.1025},
    {"Bangkok, Thailand", 13.7563, 100.5018},
    {"Singapore", 1.3521, 103.8198},
    {"Seoul, South Korea", 37.5665, 126.9780},
    {"Dubai, UAE", 25.2048, 55.2708},

    // Oceania
    {"Sydney, Australia", -33.8688, 151.2093},
    {"Melbourne, Australia", -37.8136, 144.9631},
    {"Auckland, New Zealand", -36.8485, 174.7633},
};

#endif // CONSTANTS_H
