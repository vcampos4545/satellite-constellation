#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <chrono>

class Satellite;

// Command types that can be sent to satellites
enum class SatelliteCommand
{
  SET_CONTROL_MODE,
  SET_CONTROL_ALGORITHM,
  RESET_INTEGRAL_ERROR,
  AUTO_TUNE_PID,
  DETUMBLE,
  REQUEST_TELEMETRY
};

// Telemetry packet structure
struct TelemetryPacket
{
  std::chrono::system_clock::time_point timestamp;
  std::string satelliteName;
  glm::dvec3 position;
  glm::dvec3 velocity;
  glm::dquat quaternion;
  glm::dvec3 angularVelocity;
  double altitude;
  double batteryCharge;
  double batteryPercentage;
  double powerGeneration;
  double powerConsumption;
  double propellantMass;
  bool inEclipse;
  int controlMode;
  int controlAlgorithm;
};

// Command packet structure
struct CommandPacket
{
  SatelliteCommand command;
  int intParam;
  double doubleParam;
  std::chrono::system_clock::time_point sentTime;
};

class GroundStation
{
public:
  GroundStation(const std::string &name, double latitude, double longitude);

  // Getters
  std::string getName() const { return name; }
  glm::dvec3 getPosition() const { return position; }
  std::shared_ptr<Satellite> getConnectedSatellite() const { return connectedSatellite; }
  const std::vector<std::shared_ptr<Satellite>> &getVisibleSatellites() const { return visibleSatellites; }

  // Setters
  void setConnectedSatellite(std::shared_ptr<Satellite> satellite) { connectedSatellite = satellite; }
  void addVisibleSatellite(std::shared_ptr<Satellite> satellite) { visibleSatellites.push_back(satellite); }
  void clearVisibleSatellites() { visibleSatellites.clear(); }

  // Update position based on Earth's rotation
  void updatePosition(double earthRotation, const glm::vec3 &rotationAxis);

  // Check if connected to a satellite
  bool isConnected() const { return connectedSatellite != nullptr; }

  bool isSatelliteVisible(const glm::dvec3 &satellitePos, const glm::dvec3 &earthCenter);

  // ========== COMMUNICATION FUNCTIONS ==========
  // Command uplink
  void sendCommand(std::shared_ptr<Satellite> satellite, const CommandPacket &command);
  void sendCommand(std::shared_ptr<Satellite> satellite, SatelliteCommand cmd, int intParam = 0, double doubleParam = 0.0);

  // Telemetry downlink
  void requestTelemetry(std::shared_ptr<Satellite> satellite);
  TelemetryPacket receiveTelemetry(std::shared_ptr<Satellite> satellite);

  // Communication link properties
  double calculateLinkBudget(std::shared_ptr<Satellite> satellite) const;
  double calculateDataRate(std::shared_ptr<Satellite> satellite) const;
  double getSignalStrength(std::shared_ptr<Satellite> satellite) const;

  // Get communication statistics
  const std::vector<TelemetryPacket> &getTelemetryHistory() const { return telemetryHistory; }
  int getCommandsSent() const { return commandsSent; }
  int getTelemetryReceived() const { return telemetryReceived; }
  double getTotalDataReceived() const { return totalDataReceived; } // in MB
  double getTotalDataSent() const { return totalDataSent; }         // in MB

  // Clear history
  void clearTelemetryHistory() { telemetryHistory.clear(); }

private:
  std::string name;                                          // City name
  double latitude;                                           // Latitude in degrees
  double longitude;                                          // Longitude in degrees
  glm::dvec3 position;                                       // Current world position (recalculated each frame)
  std::shared_ptr<Satellite> connectedSatellite;             // Currently connected satellite (nullptr if none)
  std::vector<std::shared_ptr<Satellite>> visibleSatellites; // List of satellites currently in view

  // ========== COMMUNICATION SYSTEM ==========
  // Antenna properties
  double antennaGain = 30.0;      // dBi - typical S-band ground station
  double transmitPower = 100.0;   // Watts
  double frequency = 2.4e9;       // Hz (S-band: 2-4 GHz)
  double systemNoiseFigure = 2.5; // dB

  // Communication statistics
  std::vector<TelemetryPacket> telemetryHistory; // History of received telemetry
  int commandsSent = 0;
  int telemetryReceived = 0;
  double totalDataReceived = 0.0; // MB
  double totalDataSent = 0.0;     // MB

  // Maximum telemetry history size
  static const size_t MAX_TELEMETRY_HISTORY = 1000;
};

#endif // GROUND_STATION_H
