#include "GroundStation.h"
#include "MathUtils.h"
#include "Constants.h"
#include "Satellite.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>

GroundStation::GroundStation(const std::string &name, double latitude, double longitude)
    : name(name), latitude(latitude), longitude(longitude), connectedSatellite(nullptr)
{
  // Calculate initial position at rotation = 0
  position = latLonToCartesian(latitude, longitude);
}

void GroundStation::update(double earthRotation, const glm::vec3 &rotationAxis)
{
  // Recalculate position from lat/lon (unrotated)
  glm::dvec3 basePosition = latLonToCartesian(latitude, longitude);

  // Apply Earth's rotation around its axis
  glm::dmat4 rotationMatrix = glm::rotate(glm::dmat4(1.0), (double)earthRotation, glm::dvec3(rotationAxis));
  position = glm::dvec3(rotationMatrix * glm::dvec4(basePosition, 1.0));
}

// Helper function to check if a ground station is visible from a satellite position
// Returns true if the satellite has line-of-sight to the ground station
bool GroundStation::isSatelliteVisible(const glm::dvec3 &satellitePos, const glm::dvec3 &earthCenter)
{
  // Vector from Earth center to ground station (surface normal)
  glm::dvec3 toStation = glm::normalize(position - earthCenter);

  // Vector from ground station to satellite
  glm::dvec3 toSatellite = glm::normalize(satellitePos - position);

  // Calculate angle between surface normal and direction to satellite
  double angleBetween = acos(glm::dot(toStation, toSatellite));

  // Satellite is visible if angle is less than 90° - minimum elevation
  // (i.e., satellite is above the horizon by at least MIN_ELEVATION)
  const double MIN_ELEVATION = glm::radians(5.0);
  const double MAX_ANGLE = PI / 2.0 - MIN_ELEVATION; // 85 degrees

  return angleBetween < MAX_ANGLE;
}

// ========== COMMUNICATION IMPLEMENTATION ==========
// These methods are commented out until they are needed

void GroundStation::sendCommand(std::shared_ptr<Satellite> satellite, const CommandPacket &command)
{
  /**
   * Send a command packet to a satellite
   * Commands are executed immediately if satellite is visible
   */

  if (!satellite)
  {
    std::cerr << "Cannot send command: satellite is null" << std::endl;
    return;
  }

  // Check if satellite is visible (line of sight)
  if (!isSatelliteVisible(satellite->getPosition(), glm::dvec3(0.0)))
  {
    std::cout << name << ": Cannot send command to " << satellite->getName()
              << " - not visible from ground station" << std::endl;
    return;
  }

  // Execute command on satellite
  switch (command.command)
  {
  case SatelliteCommand::SET_CONTROL_MODE:
    satellite->setControlMode((AttitudeControlMode)command.intParam);
    std::cout << name << ": Set " << satellite->getName() << " control mode to " << command.intParam << std::endl;
    break;

  case SatelliteCommand::SET_CONTROL_ALGORITHM:
    satellite->setControlAlgorithm((ControlAlgorithm)command.intParam);
    std::cout << name << ": Set " << satellite->getName() << " control algorithm to " << command.intParam << std::endl;
    break;

  case SatelliteCommand::RESET_INTEGRAL_ERROR:
    satellite->resetIntegralError();
    std::cout << name << ": Reset integral error for " << satellite->getName() << std::endl;
    break;

  case SatelliteCommand::AUTO_TUNE_PID:
    satellite->autoTunePID(20.0, 0.9);
    std::cout << name << ": Auto-tuned PID for " << satellite->getName() << std::endl;
    break;

  case SatelliteCommand::DETUMBLE:
    satellite->setControlMode(AttitudeControlMode::DETUMBLE);
    std::cout << name << ": Commanded " << satellite->getName() << " to detumble" << std::endl;
    break;

  case SatelliteCommand::REQUEST_TELEMETRY:
    // Telemetry request handled separately
    break;
  }

  // Update statistics
  commandsSent++;
  totalDataSent += 0.001; // ~1 KB per command
}

void GroundStation::sendCommand(std::shared_ptr<Satellite> satellite, SatelliteCommand cmd, int intParam, double doubleParam)
{
  CommandPacket packet;
  packet.command = cmd;
  packet.intParam = intParam;
  packet.doubleParam = doubleParam;
  packet.sentTime = std::chrono::system_clock::now();

  sendCommand(satellite, packet);
}

void GroundStation::requestTelemetry(std::shared_ptr<Satellite> satellite)
{
  /**
   * Request telemetry from a satellite
   * This is just a marker - telemetry is received via receiveTelemetry
   */
  CommandPacket cmd;
  cmd.command = SatelliteCommand::REQUEST_TELEMETRY;
  cmd.intParam = 0;
  cmd.doubleParam = 0.0;
  cmd.sentTime = std::chrono::system_clock::now();

  sendCommand(satellite, cmd);
}

TelemetryPacket GroundStation::receiveTelemetry(std::shared_ptr<Satellite> satellite)
{
  /**
   * Receive telemetry from a satellite
   * Creates a telemetry packet with current satellite state
   */

  TelemetryPacket packet;
  packet.timestamp = std::chrono::system_clock::now();
  packet.satelliteName = satellite->getName();
  packet.position = satellite->getPosition();
  packet.velocity = satellite->getVelocity();
  packet.quaternion = satellite->getQuaternion();
  packet.angularVelocity = satellite->getAngularVelocity();

  // Calculate altitude
  double distanceFromCenter = glm::length(satellite->getPosition());
  packet.altitude = (distanceFromCenter - EARTH_RADIUS) / 1000.0; // km

  // Power system
  packet.batteryCharge = satellite->getBatteryCharge();
  packet.batteryPercentage = satellite->getBatteryPercentage();
  packet.powerGeneration = satellite->getPowerGeneration();
  packet.powerConsumption = satellite->getPowerConsumption();
  packet.inEclipse = satellite->isInEclipse();

  // Propulsion
  packet.propellantMass = satellite->getPropellantMass();

  // Control system
  packet.controlMode = (int)satellite->getControlMode();
  packet.controlAlgorithm = (int)satellite->getControlAlgorithm();

  // Store in history
  telemetryHistory.push_back(packet);

  // Limit history size
  if (telemetryHistory.size() > MAX_TELEMETRY_HISTORY)
  {
    telemetryHistory.erase(telemetryHistory.begin());
  }

  // Update statistics
  telemetryReceived++;
  totalDataReceived += 0.010; // ~10 KB per telemetry packet

  return packet;
}

double GroundStation::calculateLinkBudget(std::shared_ptr<Satellite> satellite) const
{
  /**
   * Calculate link budget in dB
   *
   * Link Budget = Pt + Gt + Gr - Lp - Ls - Lother
   *
   * Where:
   * Pt = Transmit power (dBW)
   * Gt = Transmit antenna gain (dBi)
   * Gr = Receive antenna gain (dBi)
   * Lp = Free space path loss (dB)
   * Ls = System losses (dB)
   */

  if (!satellite)
    return -999.0;

  // Calculate range (distance) to satellite
  double range = glm::length(satellite->getPosition() - position);

  // Free space path loss: Lp = 20*log10(4*pi*d*f/c)
  const double SPEED_OF_LIGHT = 299792458.0; // m/s
  double pathLoss_dB = 20.0 * log10((4.0 * PI * range * frequency) / SPEED_OF_LIGHT);

  // Convert transmit power to dBW
  double transmitPower_dBW = 10.0 * log10(transmitPower);

  // Satellite antenna gain (assumed omnidirectional for simplicity)
  double satelliteAntennaGain_dBi = 0.0;

  // System losses (atmosphere, polarization, etc.)
  double systemLosses_dB = 3.0;

  // Link budget calculation
  double linkBudget = transmitPower_dBW + antennaGain + satelliteAntennaGain_dBi - pathLoss_dB - systemLosses_dB;

  return linkBudget;
}

double GroundStation::calculateDataRate(std::shared_ptr<Satellite> satellite) const
{
  /**
   * Calculate achievable data rate based on link budget
   *
   * Shannon capacity: C = B * log2(1 + SNR)
   *
   * Where:
   * C = Channel capacity (bits/s)
   * B = Bandwidth (Hz)
   * SNR = Signal-to-noise ratio (linear)
   */

  double linkBudget = calculateLinkBudget(satellite);

  // Estimate SNR from link budget (simplified)
  // In reality, need to account for noise temperature, bandwidth, etc.
  double SNR_dB = linkBudget - 10.0 * log10(systemNoiseFigure) - 10.0;

  // Convert SNR from dB to linear
  double SNR_linear = pow(10.0, SNR_dB / 10.0);

  // Assume 10 MHz bandwidth (typical for S-band)
  double bandwidth = 10e6; // Hz

  // Shannon capacity
  double dataRate = bandwidth * log2(1.0 + SNR_linear);

  return dataRate; // bits/s
}

double GroundStation::getSignalStrength(std::shared_ptr<Satellite> satellite) const
{
  /**
   * Get signal strength as percentage (0-100%)
   * Based on link budget
   */

  double linkBudget = calculateLinkBudget(satellite);

  // Normalize link budget to percentage
  // Typical range: -150 dB (very weak) to -50 dB (strong)
  double minLinkBudget = -150.0;
  double maxLinkBudget = -50.0;

  double percentage = 100.0 * (linkBudget - minLinkBudget) / (maxLinkBudget - minLinkBudget);

  // Clamp to 0-100%
  if (percentage < 0.0)
    percentage = 0.0;
  if (percentage > 100.0)
    percentage = 100.0;

  return percentage;
}
