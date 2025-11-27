#include "Universe.h"
#include "GroundStation.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>

// Real physical constants
const double AU = 1.496e11;           // Astronomical Unit in meters
const double G = 6.67430e-11;         // Gravitational constant
const double GEO_ALTITUDE = 35.786e6; // GEO altitude above Earth surface (meters)
const double LEO_ALTITUDE = 550e3;    // Starlink altitude ~550 km
const double PI = 3.14159265359;
const double EARTH_MASS = 5.972e24;                                  // kg
const double EARTH_RADIUS = 6.371e6;                                 // meters
const double SUN_MASS = 1.989e30;                                    // kg
const double SUN_RADIUS = 6.96e8;                                    // meters
const double MOON_MASS = 7.342e22;                                   // kg
const double MOON_RADIUS = 1.7371e6;                                 // meters
const double MOON_ORBIT_RADIUS = 3.844e8;                            // meters (384,400 km from Earth)
const double MOON_ORBITAL_PERIOD = 27.3 * 24.0 * 3600.0;             // seconds
const double MOON_ANGULAR_VELOCITY = 2.0 * PI / MOON_ORBITAL_PERIOD; // radians per second

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

    // Europe
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

    // Asia
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

// Helper function to check if a ground station is visible from a satellite position
// Returns true if the satellite has line-of-sight to the ground station
static bool isGroundStationVisible(const glm::dvec3 &satellitePos, const glm::dvec3 &groundStationPos, const glm::dvec3 &earthCenter)
{
  glm::dvec3 toEarthCenter = earthCenter - satellitePos;
  double distToCenter = glm::length(toEarthCenter);
  glm::dvec3 toGroundStation = groundStationPos - satellitePos;
  double distToGroundStation = glm::length(toGroundStation);

  // Calculate Horizon angle p
  double p = glm::asin(EARTH_RADIUS / distToCenter);

  // Calculate target angle n
  double n = glm::acos(glm::dot(toEarthCenter, toGroundStation) / (distToCenter * distToGroundStation));

  if (n < p && distToGroundStation < distToCenter)
  {
    return true;
  }

  return false;
}

// Helper function to convert latitude/longitude (in degrees) to Cartesian coordinates on Earth's surface
static glm::dvec3 latLonToCartesian(double latitudeDeg, double longitudeDeg)
{
  // Convert degrees to radians
  double latitude = latitudeDeg * PI / 180.0;
  double longitude = longitudeDeg * PI / 180.0;

  // Convert spherical coordinates to Cartesian (on Earth's surface)
  double x = EARTH_RADIUS * cos(latitude) * cos(longitude);
  double y = EARTH_RADIUS * sin(latitude);
  double z = EARTH_RADIUS * cos(latitude) * sin(longitude);

  return glm::dvec3(x, y, z);
}

// Helper function to generate any random position on Earth's surface (for initial assignment)
static glm::dvec3 generateRandomEarthSurfacePosition()
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<double> latDist(-PI / 2.0, PI / 2.0); // -90 to +90 degrees
  static std::uniform_real_distribution<double> lonDist(0.0, 2.0 * PI);       // 0 to 360 degrees

  double latitude = latDist(gen);
  double longitude = lonDist(gen);

  // Convert spherical coordinates to Cartesian (on Earth's surface)
  double x = EARTH_RADIUS * cos(latitude) * cos(longitude);
  double y = EARTH_RADIUS * sin(latitude);
  double z = EARTH_RADIUS * cos(latitude) * sin(longitude);

  return glm::dvec3(x, y, z);
}

Universe::Universe() : moonOrbitAngle(0.0)
{
}

void Universe::addBody(std::shared_ptr<CelestialBody> body)
{
  bodies.push_back(body);
}

void Universe::addSatellite(std::shared_ptr<Satellite> satellite)
{
  satellites.push_back(satellite);
}

void Universe::initializeEarthSunAndMoon()
{
  // Create Earth at origin with 23.5 degree axial tilt
  // Rotation axis tilted from Y-axis by 23.5 degrees towards the Sun (+X direction)
  float axialTiltDeg = 23.5f;
  float axialTiltRad = axialTiltDeg * PI / 180.0f;
  glm::vec3 earthRotationAxis(
      sin(axialTiltRad), // x component
      cos(axialTiltRad), // y component
      0.0f               // z component
  );

  earth = std::make_shared<CelestialBody>(
      glm::dvec3(0.0, 0.0, 0.0),
      EARTH_MASS,
      EARTH_RADIUS,
      glm::vec3(0.2f, 0.4f, 0.8f) // Bluish color
                                  // earthRotationAxis             // Tilted rotation axis
  );
  bodies.push_back(earth);

  // Create Sun at distance (1 AU away, positioned for lighting)
  sun = std::make_shared<CelestialBody>(
      glm::dvec3(AU, 0.0, 0.0),
      SUN_MASS,
      SUN_RADIUS,
      glm::vec3(1.0f, 0.9f, 0.6f) // Yellowish color
  );
  bodies.push_back(sun);

  // Create Moon orbiting Earth
  moon = std::make_shared<CelestialBody>(
      glm::dvec3(MOON_ORBIT_RADIUS, 0.0, 0.0),
      MOON_MASS,
      MOON_RADIUS,
      glm::vec3(0.7f, 0.7f, 0.7f) // Gray color
  );
  bodies.push_back(moon);

  // Generate moon's orbit path (circular path in x-z plane)
  const int numOrbitPoints = 120;
  moonOrbitPath.clear();
  for (int i = 0; i < numOrbitPoints; ++i)
  {
    double angle = (2.0 * PI * i) / numOrbitPoints;
    glm::dvec3 orbitPoint(
        MOON_ORBIT_RADIUS * cos(angle),
        0.0,
        MOON_ORBIT_RADIUS * sin(angle));
    moonOrbitPath.push_back(orbitPoint);
  }
}

void Universe::addGEOSatellite()
{
  // Calculate GEO orbit parameters
  double orbitalRadius = EARTH_RADIUS + GEO_ALTITUDE;            // Distance from Earth center
  double orbitalVelocity = sqrt(G * EARTH_MASS / orbitalRadius); // Circular orbit velocity

  // Start satellite at (orbital radius, 0, 0) with velocity in +Z direction
  glm::dvec3 position(orbitalRadius, 0.0, 0.0);
  glm::dvec3 velocity(0.0, 0.0, -orbitalVelocity);

  // Create satellite with bright color for visibility
  auto satellite = std::make_shared<Satellite>(
      position,
      velocity,
      glm::vec3(0.3f, 0.9f, 1.0f),
      -1, // planeId (-1 for GEO satellites)
      0   // indexInPlane
  );

  // Calculate complete orbital path
  satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), 120);

  satellites.push_back(satellite);
}

void Universe::addStarlinkConstellation(int numPlanes, int satellitesPerPlane)
{
  // Starlink-like parameters
  double orbitalRadius = EARTH_RADIUS + LEO_ALTITUDE;
  double orbitalVelocity = sqrt(G * EARTH_MASS / orbitalRadius);
  double inclination = 53.0 * PI / 180.0; // 53 degrees inclination (typical for Starlink)

  // Create satellites in multiple orbital planes
  for (int plane = 0; plane < numPlanes; ++plane)
  {
    // Right ascension of ascending node (RAAN) - evenly distribute planes
    double raan = (2.0 * PI * plane) / numPlanes;

    // Create satellites in this plane
    for (int sat = 0; sat < satellitesPerPlane; ++sat)
    {
      // True anomaly - position along the orbit
      double trueAnomaly = (2.0 * PI * sat) / satellitesPerPlane;

      // Start with position in orbital plane (x-z plane, circular orbit)
      glm::dvec3 position(
          orbitalRadius * cos(trueAnomaly),
          0.0,
          orbitalRadius * sin(trueAnomaly));

      // Velocity perpendicular to position (tangent to orbit)
      glm::dvec3 velocity(
          -orbitalVelocity * sin(trueAnomaly),
          0.0,
          orbitalVelocity * cos(trueAnomaly));

      // Apply inclination (rotate around x-axis)
      glm::dmat4 inclinationMatrix = glm::rotate(glm::dmat4(1.0), inclination, glm::dvec3(1.0, 0.0, 0.0));
      position = glm::dvec3(inclinationMatrix * glm::dvec4(position, 1.0));
      velocity = glm::dvec3(inclinationMatrix * glm::dvec4(velocity, 0.0));

      // Apply RAAN (rotate around y-axis)
      glm::dmat4 raanMatrix = glm::rotate(glm::dmat4(1.0), raan, glm::dvec3(0.0, 1.0, 0.0));
      position = glm::dvec3(raanMatrix * glm::dvec4(position, 1.0));
      velocity = glm::dvec3(raanMatrix * glm::dvec4(velocity, 0.0));

      // Create satellite with bright cyan/white color for better visibility
      auto satellite = std::make_shared<Satellite>(
          position,
          velocity,
          glm::vec3(0.3f, 0.9f, 1.0f), // Bright cyan color
          plane,                       // planeId
          sat                          // indexInPlane
      );

      // Calculate complete orbital path
      satellite->calculateFullOrbit(earth->getPosition(), earth->getMass(), 120);
      satellites.push_back(satellite);
    }
  }
}

void Universe::addGroundStations()
{
  // Add ground stations at major city locations
  const int numCities = sizeof(MAJOR_CITIES) / sizeof(MAJOR_CITIES[0]);

  for (int i = 0; i < numCities; ++i)
  {
    const City &city = MAJOR_CITIES[i];
    glm::dvec3 position = latLonToCartesian(city.latitude, city.longitude);
    auto groundStation = std::make_shared<GroundStation>(position);
    groundStations.push_back(groundStation);
  }

  std::cout << "Added " << numCities << " ground stations at major city locations" << std::endl;
}

void Universe::update(double deltaTime, double maxPhysicsStep)
{
  moonOrbitAngle += MOON_ANGULAR_VELOCITY * deltaTime;

  // Update moon's position (circular orbit around Earth in x-z plane)
  if (moon)
  {
    glm::dvec3 earthPos = earth->getPosition();
    moon->setPosition(glm::dvec3(
        earthPos.x + MOON_ORBIT_RADIUS * cos(moonOrbitAngle),
        earthPos.y,
        earthPos.z + MOON_ORBIT_RADIUS * sin(moonOrbitAngle)));
  }

  // Use sub-stepping to keep physics stable even with large time warps
  // Break large time steps into smaller chunks
  double remainingTime = deltaTime;

  while (remainingTime > 0.0)
  {
    // Take the smaller of: remaining time or max physics step
    double stepTime = std::min(remainingTime, maxPhysicsStep);

    // Update all satellites with this smaller step
    for (auto &satellite : satellites)
    {
      satellite->update(stepTime, earth->getPosition(), earth->getMass());
    }

    remainingTime -= stepTime;
  }

  // TODO: Implement this Update ground station connections
  for (auto &groundStation : groundStations)
  {
    // Check if currently connected satellite is still visible
    auto currentSat = groundStation->getConnectedSatellite();
    bool needNewConnection = true;

    if (currentSat)
    {
      // Check if current satellite is still visible
      if (isGroundStationVisible(currentSat->getPosition(), groundStation->getPosition(), earth->getPosition()))
      {
        // Still visible, keep the connection
        needNewConnection = false;
      }
    }

    if (needNewConnection)
    {
      // Find a new visible satellite
      std::shared_ptr<Satellite> bestSatellite = nullptr;
      double closestDistance = 1e20; // Very large number

      for (auto &satellite : satellites)
      {
        if (isGroundStationVisible(satellite->getPosition(), groundStation->getPosition(), earth->getPosition()))
        {
          // This satellite is visible, check if it's closer than previous best
          double distance = glm::length(satellite->getPosition() - groundStation->getPosition());
          if (distance < closestDistance)
          {
            closestDistance = distance;
            bestSatellite = satellite;
          }
        }
      }

      // Connect to the best (closest) visible satellite, or disconnect if none are visible
      groundStation->setConnectedSatellite(bestSatellite);
    }
  }
}
