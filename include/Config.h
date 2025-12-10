#ifndef CONFIG_H
#define CONFIG_H

// Structure to hold orbit information
struct Orbit
{
  double a;     // Semimajor axis (meters)
  double e;     // Eccentricity
  double i;     // Inclination (degrees)
  double omega; // Right ascension of ascending node (Longitude of ascending node)
  double w;     // Argument of perigee/periapsis
  double v;     // True anomoly
};

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

#endif // CONFIG_H
