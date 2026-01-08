#include "CommLink.h"
#include "Satellite.h"
#include "GroundStation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include "Constants.h"

// Physical constants

// Helper: Convert dBm to linear Watts
static double dBm_to_watts(double dBm)
{
  return pow(10.0, (dBm - 30.0) / 10.0);
}

// Helper: Convert linear Watts to dBm
static double watts_to_dBm(double watts)
{
  return 10.0 * log10(watts) + 30.0;
}

// Helper: Convert linear to dB
static double linear_to_dB(double linear)
{
  return 10.0 * log10(linear);
}

// Helper: Convert dB to linear
static double dB_to_linear(double dB)
{
  return pow(10.0, dB / 10.0);
}

CommLink::CommLink(Satellite *satellite, GroundStation *groundStation)
    : satellite(satellite), groundStation(groundStation)
{
}

void CommLink::update(double currentTime, double deltaTime)
{
  // Calculate downlink budget (satellite -> ground)
  downlinkBudget = calculateLinkBudget(false);

  // Calculate uplink budget (ground -> satellite)
  uplinkBudget = calculateLinkBudget(true);

  // Process message queue (deliver messages that have arrived)
  processMessages(currentTime);

  // Simulate continuous data transfer if link is active
  if (downlinkBudget.linkActive && downlinkBudget.currentDataRate_bps > 0)
  {
    // Calculate bytes transferred this timestep
    double bitsTransferred = downlinkBudget.currentDataRate_bps * deltaTime;
    double bytesTransferred = bitsTransferred / 8.0;
    totalDataTransferred += bytesTransferred;
  }
}

LinkBudget CommLink::calculateLinkBudget(bool isUplink) const
{
  LinkBudget budget;

  // Get satellite and ground station positions/velocities
  glm::dvec3 satPos = satellite->getPosition();
  glm::dvec3 satVel = satellite->getVelocity();
  glm::dvec3 gsPos = groundStation->getPosition();
  glm::dvec3 gsVel(0.0); // Ground station velocity (could add Earth rotation later)

  // ========== GEOMETRY ==========
  glm::dvec3 r_vec = isUplink ? (satPos - gsPos) : (gsPos - satPos);
  budget.distance = glm::length(r_vec);
  glm::dvec3 losVector = glm::normalize(r_vec);

  // Calculate elevation angle from ground station
  glm::dvec3 gsUp = glm::normalize(gsPos); // Vector from Earth center to GS
  double elevationAngle = asin(glm::dot(losVector, gsUp));
  budget.elevation = elevationAngle;

  // Propagation delay
  budget.propagationDelay = budget.distance / SPEED_OF_LIGHT;

  // Check line of sight
  budget.hasLineOfSight = checkLineOfSight(satPos, gsPos, glm::dvec3(0.0), EARTH_RADIUS);

  if (!budget.hasLineOfSight)
  {
    budget.linkActive = false;
    return budget; // No LOS, link impossible
  }

  // ========== POWER BUDGET ==========

  // Select transmitter and receiver parameters based on link direction
  double txPower_dBm, txGain_dBi, rxGain_dBi;
  double txBeamwidth_deg, rxBeamwidth_deg;
  glm::dvec3 txAntennaAxis, rxAntennaAxis;

  if (isUplink)
  {
    // Ground -> Satellite
    txPower_dBm = gsTxPower_dBm;
    txGain_dBi = gsTxGain_dBi;
    rxGain_dBi = satRxGain_dBi;
    txBeamwidth_deg = gsBeamwidth_deg;
    rxBeamwidth_deg = satBeamwidth_deg;

    // Ground station antenna points at satellite
    txAntennaAxis = glm::normalize(satPos - gsPos);

    // Satellite antenna (simplified: assume nadir-pointing or omnidirectional)
    rxAntennaAxis = glm::normalize(-satPos); // Nadir pointing
  }
  else
  {
    // Satellite -> Ground (downlink)
    txPower_dBm = satTxPower_dBm;
    txGain_dBi = satTxGain_dBi;
    rxGain_dBi = gsRxGain_dBi;
    txBeamwidth_deg = satBeamwidth_deg;
    rxBeamwidth_deg = gsBeamwidth_deg;

    // Satellite antenna (nadir-pointing)
    txAntennaAxis = glm::normalize(-satPos); // Nadir

    // Ground station antenna points at satellite
    rxAntennaAxis = glm::normalize(satPos - gsPos);
  }

  budget.transmitPower_dBm = txPower_dBm;
  budget.transmitGain_dBi = txGain_dBi;
  budget.receiveGain_dBi = rxGain_dBi;

  // ========== LOSSES ==========

  // Free-space path loss (Friis equation)
  budget.pathLoss_dB = calculatePathLoss(budget.distance, frequency);

  // Atmospheric attenuation (only applies below ~100 km altitude)
  if (elevationAngle > 0) // Only calculate if satellite is above horizon
  {
    budget.atmosphericLoss_dB = calculateAtmosphericLoss(elevationAngle, frequency);
  }
  else
  {
    budget.atmosphericLoss_dB = 1000.0; // Effectively infinite loss below horizon
  }

  // Antenna pointing loss (transmit side)
  glm::dvec3 targetDirection = isUplink ? glm::normalize(satPos - gsPos) : glm::normalize(gsPos - satPos);
  budget.pointingLoss_dB = calculatePointingLoss(txAntennaAxis, targetDirection, txBeamwidth_deg);

  // Polarization loss (assume aligned for now - could add mismatch)
  budget.polarizationLoss_dB = 0.5; // Typical 0.5 dB for slight mismatch

  // ========== RECEIVED POWER ==========
  // Friis Transmission Equation in dB:
  // P_r (dBm) = P_t (dBm) + G_t (dBi) + G_r (dBi) - Losses (dB)
  budget.receivedPower_dBm = budget.transmitPower_dBm +
                             budget.transmitGain_dBi +
                             budget.receiveGain_dBi -
                             budget.pathLoss_dB -
                             budget.atmosphericLoss_dB -
                             budget.pointingLoss_dB -
                             budget.polarizationLoss_dB;

  // ========== NOISE AND SNR ==========
  budget.noiseFloor_dBm = calculateNoiseFloor(bandwidth, noiseFigure_dB, systemTemp_K);
  budget.snr_dB = budget.receivedPower_dBm - budget.noiseFloor_dBm;

  // Link margin (how much SNR above minimum threshold)
  budget.linkMargin_dB = budget.snr_dB - minSNRThreshold_dB;
  budget.linkActive = (budget.linkMargin_dB > 0.0);

  // ========== DATA RATE ==========
  if (budget.linkActive)
  {
    double snr_linear = dB_to_linear(budget.snr_dB);

    // Shannon capacity (theoretical maximum)
    budget.maxDataRate_bps = calculateShannonCapacity(bandwidth, snr_linear);

    // Actual data rate using selected modulation
    budget.currentDataRate_bps = calculateModulationDataRate(bandwidth, budget.snr_dB, modulation);
  }
  else
  {
    budget.maxDataRate_bps = 0.0;
    budget.currentDataRate_bps = 0.0;
  }

  // ========== DOPPLER SHIFT ==========
  budget.dopplerShift_Hz = calculateDopplerShift(satVel, gsVel, losVector, frequency);

  return budget;
}

double CommLink::calculatePathLoss(double distance, double frequency) const
{
  /**
   * Free-Space Path Loss (FSPL)
   *
   * Derivation from Friis Transmission Equation:
   * P_r / P_t = G_t * G_r * (λ / 4πd)²
   *
   * Path Loss L = P_t / P_r = (4πd / λ)²
   *
   * In dB: L (dB) = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
   *             = 20*log10(d) + 20*log10(f) - 147.55
   *
   * where:
   * - d is distance in meters
   * - f is frequency in Hz
   * - c is speed of light (299792458 m/s)
   * - Constant: 20*log10(4π/c) = -147.55
   */

  if (distance <= 0.0 || frequency <= 0.0)
  {
    return 1000.0; // Invalid parameters - return huge loss
  }

  double pathLoss_dB = 20.0 * log10(distance) +
                       20.0 * log10(frequency) -
                       147.55;

  return pathLoss_dB;
}

double CommLink::calculateAtmosphericLoss(double elevationAngle, double frequency) const
{
  /**
   * Atmospheric Attenuation (Simplified ITU-R P.676 model)
   *
   * Atmosphere absorbs RF energy due to:
   * - Oxygen (O2) resonance around 60 GHz
   * - Water vapor (H2O) absorption
   * - Rain (significant above 10 GHz)
   *
   * Simplified model:
   * - Zenith attenuation (90° elevation) depends on frequency
   * - Slant path attenuation = zenith_atten / sin(elevation)
   */

  // Zenith attenuation (dB) for dry atmosphere at sea level
  // Empirical fit to ITU-R P.676 for frequencies 1-40 GHz
  double freq_GHz = frequency / 1e9;
  double zenithAttenuation_dB;

  if (freq_GHz < 1.0)
  {
    zenithAttenuation_dB = 0.01; // Negligible at VHF/UHF
  }
  else if (freq_GHz < 10.0)
  {
    zenithAttenuation_dB = 0.01 + 0.002 * (freq_GHz - 1.0); // ~0.03 dB at 10 GHz
  }
  else if (freq_GHz < 40.0)
  {
    zenithAttenuation_dB = 0.03 + 0.01 * (freq_GHz - 10.0); // ~0.3 dB at 40 GHz
  }
  else
  {
    zenithAttenuation_dB = 0.3; // Cap at 40 GHz
  }

  // Slant path correction
  // At low elevations, signal travels through more atmosphere
  if (elevationAngle > 0.0)
  {
    double slantPathFactor = 1.0 / sin(elevationAngle);
    // Clamp to avoid huge losses at very low elevations (atmosphere isn't infinite)
    slantPathFactor = std::min(slantPathFactor, 10.0);

    return zenithAttenuation_dB * slantPathFactor;
  }
  else
  {
    return 1000.0; // Below horizon - effectively infinite loss
  }
}

double CommLink::calculatePointingLoss(const glm::dvec3 &antennaAxis,
                                       const glm::dvec3 &targetDirection,
                                       double beamwidth_deg) const
{
  /**
   * Antenna Pointing Loss (Gaussian Beam Pattern)
   *
   * Real antenna gain patterns are complex, but can be approximated as Gaussian:
   *
   * G(θ) = G_max * exp(-(θ / θ_0)²)
   *
   * In dB: G(θ) = G_max - 12 * (θ / θ_3dB)²
   *
   * where:
   * - θ is off-boresight angle
   * - θ_3dB is the 3-dB beamwidth (half-power beamwidth)
   *
   * Pointing loss is the reduction in gain due to misalignment:
   * Pointing Loss (dB) = 12 * (θ / θ_3dB)²
   */

  // Calculate angle between antenna boresight and target
  double cosAngle = glm::dot(glm::normalize(antennaAxis), glm::normalize(targetDirection));
  cosAngle = glm::clamp(cosAngle, -1.0, 1.0); // Numerical safety
  double angle_rad = acos(cosAngle);
  double angle_deg = angle_rad * 180.0 / PI;

  // Convert 3-dB beamwidth to half-power beamwidth
  double theta_3dB_rad = beamwidth_deg * PI / 180.0;

  // Gaussian beam pattern loss
  double pointingLoss_dB = 12.0 * pow(angle_rad / theta_3dB_rad, 2.0);

  // Clamp to reasonable values (avoid infinite loss)
  pointingLoss_dB = std::min(pointingLoss_dB, 50.0);

  return pointingLoss_dB;
}

double CommLink::calculateDopplerShift(const glm::dvec3 &satVelocity,
                                       const glm::dvec3 &gsVelocity,
                                       const glm::dvec3 &losVector,
                                       double frequency) const
{
  /**
   * Doppler Shift
   *
   * When transmitter and receiver have relative velocity, the received
   * frequency shifts:
   *
   * f_received = f_transmitted * (1 + v_radial / c)
   *
   * For small velocities (v << c):
   * Δf ≈ f * (v_radial / c)
   *
   * where:
   * - v_radial = (v_sat - v_gs) · ĥ (line-of-sight component)
   * - Positive v_radial = approaching (blue shift, higher frequency)
   * - Negative v_radial = receding (red shift, lower frequency)
   */

  glm::dvec3 relativeVelocity = satVelocity - gsVelocity;
  double radialVelocity = glm::dot(relativeVelocity, losVector);

  // Doppler shift (Hz)
  double dopplerShift = frequency * (radialVelocity / SPEED_OF_LIGHT);

  return dopplerShift;
}

double CommLink::calculateNoiseFloor(double bandwidth, double noiseFigure, double systemTemp) const
{
  /**
   * Noise Floor Calculation
   *
   * Thermal noise power in a receiver:
   * N = k * T * B (Watts)
   *
   * where:
   * - k = Boltzmann constant (1.380649e-23 J/K)
   * - T = system noise temperature (K)
   * - B = bandwidth (Hz)
   *
   * In dBm:
   * N (dBm) = 10*log10(kTB / 1mW)
   *         = 10*log10(kTB) + 30
   *         = 10*log10(k) + 10*log10(T) + 10*log10(B) + 30
   *         = -228.6 + 10*log10(T) + 10*log10(B) + 30
   *         = -198.6 + 10*log10(T) + 10*log10(B)
   *
   * For T = 290 K (room temperature):
   * N (dBm/Hz) = -174 dBm/Hz
   *
   * Total noise including receiver noise figure:
   * N_total (dBm) = -174 + 10*log10(B) + NF
   */

  double kTB_watts = BOLTZMANN_CONSTANT * systemTemp * bandwidth;
  double noise_dBm = watts_to_dBm(kTB_watts);

  // Add receiver noise figure
  noise_dBm += noiseFigure;

  return noise_dBm;
}

double CommLink::calculateShannonCapacity(double bandwidth, double snr_linear) const
{
  /**
   * Shannon-Hartley Theorem
   *
   * The theoretical maximum data rate for a given bandwidth and SNR:
   *
   * C = B * log2(1 + SNR)
   *
   * where:
   * - C = channel capacity (bits/second)
   * - B = bandwidth (Hz)
   * - SNR = signal-to-noise ratio (linear, not dB)
   *
   * This is the theoretical limit - practical systems achieve 50-80% of this.
   */

  if (snr_linear <= 0.0)
  {
    return 0.0;
  }

  double capacity_bps = bandwidth * log2(1.0 + snr_linear);
  return capacity_bps;
}

double CommLink::calculateModulationDataRate(double bandwidth, double snr_dB, ModulationType modulation) const
{
  /**
   * Modulation-Specific Data Rate
   *
   * Different modulation schemes have different spectral efficiencies
   * (bits per symbol) and SNR requirements.
   *
   * Spectral Efficiency (bits/symbol):
   * - BPSK: 1 bit/symbol
   * - QPSK: 2 bits/symbol
   * - 16-QAM: 4 bits/symbol
   * - 64-QAM: 6 bits/symbol
   *
   * Data rate = Symbol Rate * Bits/Symbol
   *           ≈ Bandwidth * Bits/Symbol (for Nyquist filtering)
   *
   * Minimum SNR requirements (approx for BER < 1e-5):
   * - BPSK: 9.6 dB
   * - QPSK: 9.6 dB (same as BPSK but 2x data rate)
   * - 16-QAM: 16.5 dB
   * - 64-QAM: 22.5 dB
   */

  double bitsPerSymbol;
  double minSNR_dB;

  switch (modulation)
  {
  case ModulationType::BPSK:
    bitsPerSymbol = 1.0;
    minSNR_dB = 9.6;
    break;
  case ModulationType::QPSK:
    bitsPerSymbol = 2.0;
    minSNR_dB = 9.6;
    break;
  case ModulationType::QAM16:
    bitsPerSymbol = 4.0;
    minSNR_dB = 16.5;
    break;
  case ModulationType::QAM64:
    bitsPerSymbol = 6.0;
    minSNR_dB = 22.5;
    break;
  default:
    bitsPerSymbol = 2.0;
    minSNR_dB = 9.6;
  }

  // Check if SNR is sufficient for this modulation
  if (snr_dB < minSNR_dB)
  {
    return 0.0; // SNR too low for this modulation
  }

  // Data rate = Bandwidth * Spectral Efficiency
  // Apply 0.8 factor for practical implementations (overhead, coding, etc.)
  double dataRate_bps = bandwidth * bitsPerSymbol * 0.8;

  return dataRate_bps;
}

double CommLink::calculateBER(double snr_linear, ModulationType modulation) const
{
  /**
   * Bit Error Rate (BER) Calculation
   *
   * BER depends on modulation and SNR. Approximate formulas:
   *
   * BPSK/QPSK: BER ≈ 0.5 * erfc(√(Eb/N0))
   * where Eb/N0 = SNR * (Symbol Rate / Bit Rate)
   *
   * For BPSK: Eb/N0 = SNR
   * For QPSK: Eb/N0 = SNR / 2
   */

  if (snr_linear <= 0.0)
  {
    return 0.5; // 50% error rate - completely random
  }

  double ber;

  switch (modulation)
  {
  case ModulationType::BPSK:
    ber = 0.5 * erfc(sqrt(snr_linear));
    break;

  case ModulationType::QPSK:
    ber = 0.5 * erfc(sqrt(snr_linear / 2.0));
    break;

  case ModulationType::QAM16:
  {
    // Approximate formula for 16-QAM
    double sqrtSNR = sqrt(snr_linear / 10.0);
    ber = 0.75 * erfc(sqrtSNR);
    break;
  }

  case ModulationType::QAM64:
  {
    // Approximate formula for 64-QAM
    double sqrtSNR = sqrt(snr_linear / 42.0);
    ber = 0.9167 * erfc(sqrtSNR);
    break;
  }

  default:
    ber = 0.5 * erfc(sqrt(snr_linear / 2.0));
  }

  return ber;
}

bool CommLink::checkLineOfSight(const glm::dvec3 &satPos, const glm::dvec3 &gsPos,
                                const glm::dvec3 &earthCenter, double earthRadius) const
{
  /**
   * Line-of-Sight Check (Ray-Sphere Intersection)
   *
   * Check if Earth blocks the signal path between satellite and ground station.
   *
   * Ray from ground station toward satellite:
   * P(t) = gsPos + t * (satPos - gsPos)
   *
   * Earth sphere: |P - earthCenter|² = R²
   *
   * Solve for intersection:
   * |gsPos + t*d - earthCenter|² = R²
   *
   * where d = (satPos - gsPos)
   *
   * Quadratic equation: at² + bt + c = 0
   * a = d·d
   * b = 2 * (gsPos - earthCenter)·d
   * c = |gsPos - earthCenter|² - R²
   *
   * If discriminant < 0: no intersection (clear line of sight)
   * If discriminant ≥ 0: intersection exists (Earth blocks signal)
   *
   * Special case: If ground station is on Earth surface and satellite is above,
   * we need to check if ray goes through Earth between them.
   */

  glm::dvec3 d = satPos - gsPos;
  glm::dvec3 f = gsPos - earthCenter;

  double a = glm::dot(d, d);
  double b = 2.0 * glm::dot(f, d);
  double c = glm::dot(f, f) - earthRadius * earthRadius;

  double discriminant = b * b - 4.0 * a * c;

  if (discriminant < 0.0)
  {
    // No intersection - clear line of sight
    return true;
  }

  // Check if intersection is between ground station and satellite
  double t1 = (-b - sqrt(discriminant)) / (2.0 * a);
  double t2 = (-b + sqrt(discriminant)) / (2.0 * a);

  // If both intersections are before the satellite (t < 1), path is blocked
  // If both are behind the ground station (t < 0), path is clear
  if (t2 < 0.0 || t1 > 1.0)
  {
    return true; // Clear line of sight
  }

  return false; // Earth blocks the path
}

void CommLink::sendMessage(const std::string &message, size_t sizeBytes, bool isUplink, double currentTime)
{
  // Calculate propagation delay
  const LinkBudget &budget = isUplink ? uplinkBudget : downlinkBudget;

  if (!budget.linkActive)
  {
    // Link not active - message cannot be sent
    // In a real system, this would go into a retry queue
    return;
  }

  double arrivalTime = currentTime + budget.propagationDelay;

  // Add message to queue
  messageQueue.push(CommMessage(message, sizeBytes, currentTime, arrivalTime, isUplink));
}

void CommLink::processMessages(double currentTime)
{
  // Clear previously delivered messages
  deliveredMessages.clear();

  // Process messages in queue
  while (!messageQueue.empty())
  {
    const CommMessage &msg = messageQueue.front();

    if (msg.arrivalTime <= currentTime)
    {
      // Message has arrived - deliver it
      deliveredMessages.push_back(msg);
      messageQueue.pop();
    }
    else
    {
      // Messages are ordered by arrival time, so we can stop
      break;
    }
  }
}

std::vector<CommMessage> CommLink::getDeliveredMessages()
{
  std::vector<CommMessage> messages = deliveredMessages;
  deliveredMessages.clear();
  return messages;
}
