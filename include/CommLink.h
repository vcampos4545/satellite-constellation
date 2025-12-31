#ifndef COMMLINK_H
#define COMMLINK_H

#include <glm/glm.hpp>
#include <memory>
#include <queue>
#include <chrono>
#include <string>
#include "Constants.h"

class Satellite;
class GroundStation;

// ========== RF LINK BUDGET PARAMETERS ==========

// Modulation schemes (affects spectral efficiency and BER)
enum class ModulationType
{
  BPSK,  // Binary Phase Shift Keying - 1 bit/symbol, robust
  QPSK,  // Quadrature PSK - 2 bits/symbol
  QAM16, // 16-QAM - 4 bits/symbol
  QAM64  // 64-QAM - 6 bits/symbol, high data rate but needs good SNR
};

// Frequency bands
enum class FrequencyBand
{
  VHF,   // 30-300 MHz (amateur radio, cubesats)
  UHF,   // 300-3000 MHz (common for LEO satellites)
  S_BAND, // 2-4 GHz (deep space, GPS)
  X_BAND, // 8-12 GHz (high data rate downlink)
  Ka_BAND // 26-40 GHz (very high data rate, weather sensitive)
};

// Message packet structure (delayed delivery simulation)
struct CommMessage
{
  std::string content;
  size_t sizeBytes;
  double sentTime;
  double arrivalTime;
  bool isUplink; // true = ground->sat, false = sat->ground

  CommMessage(const std::string &msg, size_t size, double sent, double arrival, bool uplink)
      : content(msg), sizeBytes(size), sentTime(sent), arrivalTime(arrival), isUplink(uplink) {}
};

// Link budget result structure
struct LinkBudget
{
  // Geometric parameters
  double distance;          // meters
  double elevation;         // radians (from ground station horizon)
  double propagationDelay;  // seconds
  bool hasLineOfSight;      // Earth not blocking path

  // Power budget (all in dB)
  double transmitPower_dBm;
  double transmitGain_dBi;
  double receiveGain_dBi;
  double pathLoss_dB;
  double atmosphericLoss_dB;
  double pointingLoss_dB;
  double polarizationLoss_dB;
  double receivedPower_dBm;

  // Noise and quality
  double noiseFloor_dBm;
  double snr_dB;
  double linkMargin_dB;
  bool linkActive;

  // Data transfer
  double maxDataRate_bps;    // Maximum achievable data rate
  double currentDataRate_bps; // Actual data rate (may be lower due to modulation)
  double dopplerShift_Hz;    // Frequency shift due to relative motion

  // Constructor with defaults
  LinkBudget() : distance(0), elevation(0), propagationDelay(0), hasLineOfSight(false),
                 transmitPower_dBm(0), transmitGain_dBi(0), receiveGain_dBi(0),
                 pathLoss_dB(0), atmosphericLoss_dB(0), pointingLoss_dB(0),
                 polarizationLoss_dB(0), receivedPower_dBm(-200),
                 noiseFloor_dBm(-100), snr_dB(-200), linkMargin_dB(-200),
                 linkActive(false), maxDataRate_bps(0), currentDataRate_bps(0),
                 dopplerShift_Hz(0) {}
};

/**
 * CommLink - RF Communication Link Simulator
 *
 * Implements realistic electromagnetic wave propagation and link budget analysis
 * following industry-standard models (Friis equation, Shannon-Hartley, ITU atmospheric)
 *
 * Key Features:
 * - Friis transmission equation for path loss
 * - Antenna pointing loss with Gaussian beam pattern
 * - Atmospheric attenuation (ITU-R P.676 model)
 * - Doppler shift calculation
 * - Shannon capacity and modulation-specific data rates
 * - Realistic message queueing with propagation delay
 */
class CommLink
{
public:
  CommLink(Satellite *satellite, GroundStation *groundStation);

  // Main update function - call every simulation timestep
  void update(double currentTime, double deltaTime);

  // ========== LINK BUDGET CALCULATION ==========
  LinkBudget calculateLinkBudget(bool isUplink) const;

  // Calculate free-space path loss (dB)
  // Uses: Path Loss (dB) = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
  double calculatePathLoss(double distance, double frequency) const;

  // Calculate atmospheric attenuation (ITU-R P.676 model)
  double calculateAtmosphericLoss(double elevationAngle, double frequency) const;

  // Calculate antenna pointing loss (Gaussian beam pattern)
  double calculatePointingLoss(const glm::dvec3 &antennaAxis, const glm::dvec3 &targetDirection,
                               double beamwidth) const;

  // Calculate Doppler shift due to relative velocity
  double calculateDopplerShift(const glm::dvec3 &satVelocity, const glm::dvec3 &gsVelocity,
                              const glm::dvec3 &losVector, double frequency) const;

  // Calculate noise floor: N = kTB (in dBm)
  // k = Boltzmann constant, T = system temp, B = bandwidth
  double calculateNoiseFloor(double bandwidth, double noiseFigure, double systemTemp = 290.0) const;

  // Calculate maximum data rate using Shannon-Hartley theorem
  // C = B * log2(1 + SNR)
  double calculateShannonCapacity(double bandwidth, double snr_linear) const;

  // Calculate achievable data rate for specific modulation
  double calculateModulationDataRate(double bandwidth, double snr_dB, ModulationType modulation) const;

  // Calculate bit error rate for given modulation and SNR
  double calculateBER(double snr_linear, ModulationType modulation) const;

  // ========== LINE OF SIGHT ==========
  bool checkLineOfSight(const glm::dvec3 &satPos, const glm::dvec3 &gsPos,
                        const glm::dvec3 &earthCenter, double earthRadius) const;

  // ========== MESSAGE PASSING ==========
  // Send message (uplink or downlink)
  void sendMessage(const std::string &message, size_t sizeBytes, bool isUplink, double currentTime);

  // Process message queue and deliver messages that have arrived
  void processMessages(double currentTime);

  // Get delivered messages (then clears the delivered queue)
  std::vector<CommMessage> getDeliveredMessages();

  // ========== GETTERS ==========
  const LinkBudget &getDownlinkBudget() const { return downlinkBudget; }
  const LinkBudget &getUplinkBudget() const { return uplinkBudget; }
  bool isLinkActive() const { return downlinkBudget.linkActive || uplinkBudget.linkActive; }
  double getCurrentDataRate() const { return downlinkBudget.currentDataRate_bps; }
  int getPendingMessageCount() const { return messageQueue.size(); }
  double getTotalDataTransferred() const { return totalDataTransferred; } // bytes

  // ========== CONFIGURATION ==========
  void setFrequency(double freq) { frequency = freq; }
  void setBandwidth(double bw) { bandwidth = bw; }
  void setModulation(ModulationType mod) { modulation = mod; }
  void setMinSNRThreshold(double snr_dB) { minSNRThreshold_dB = snr_dB; }

private:
  // Link endpoints
  Satellite *satellite;
  GroundStation *groundStation;

  // RF parameters
  double frequency = 2.4e9;        // Hz (default S-band: 2.4 GHz)
  double bandwidth = 10e6;         // Hz (10 MHz)
  ModulationType modulation = ModulationType::QPSK;
  double minSNRThreshold_dB = 3.0; // Minimum SNR for link to be considered active

  // Satellite transmitter (downlink)
  double satTxPower_dBm = 30.0;   // 30 dBm = 1 W
  double satTxGain_dBi = 3.0;     // Small patch antenna
  double satRxGain_dBi = 3.0;     // Same antenna for receive
  double satBeamwidth_deg = 60.0; // Wide beam (omnidirectional-ish)

  // Ground station (uplink)
  double gsTxPower_dBm = 40.0;    // 40 dBm = 10 W
  double gsTxGain_dBi = 30.0;     // Directional dish antenna
  double gsRxGain_dBi = 30.0;     // Same dish for receive
  double gsBeamwidth_deg = 3.0;   // Narrow beam (tracking required)

  // System noise
  double noiseFigure_dB = 2.5;     // Receiver noise figure
  double systemTemp_K = 290.0;     // System noise temperature (room temp)

  // Link budgets (updated each frame)
  LinkBudget downlinkBudget; // Satellite -> Ground
  LinkBudget uplinkBudget;   // Ground -> Satellite

  // Message queues
  std::queue<CommMessage> messageQueue;        // Messages in transit
  std::vector<CommMessage> deliveredMessages;  // Messages that arrived this timestep

  // Statistics
  double totalDataTransferred = 0.0; // bytes
};

#endif // COMMLINK_H
