#ifndef ALERTSYSTEM_H
#define ALERTSYSTEM_H

#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

// Alert severity levels
enum class AlertSeverity
{
  INFO,     // Informational message
  WARNING,  // Warning condition (yellow)
  CRITICAL, // Critical condition (red)
  NOMINAL   // System nominal (green) - used for status
};

// Alert categories
enum class AlertCategory
{
  POWER,        // Battery, solar panels, power generation
  THERMAL,      // Temperature limits
  ADCS,         // Attitude control issues
  COMMS,        // Communication link issues
  ORBIT,        // Orbital parameters out of spec
  PROPULSION,   // Propellant, thrusters
  GENERAL       // General system alerts
};

// Individual alert structure
struct Alert
{
  std::string message;
  AlertSeverity severity;
  AlertCategory category;
  std::chrono::system_clock::time_point timestamp;
  bool acknowledged;

  Alert(const std::string &msg, AlertSeverity sev, AlertCategory cat)
      : message(msg), severity(sev), category(cat), acknowledged(false)
  {
    timestamp = std::chrono::system_clock::now();
  }
};

// Alert system for tracking satellite health
class AlertSystem
{
public:
  AlertSystem() : maxAlerts(50) {}

  // Add a new alert
  void addAlert(const std::string &message, AlertSeverity severity, AlertCategory category)
  {
    alerts.push_back(Alert(message, severity, category));

    // Keep only the most recent maxAlerts
    if (alerts.size() > maxAlerts)
    {
      alerts.erase(alerts.begin());
    }
  }

  // Clear all alerts
  void clearAlerts()
  {
    alerts.clear();
  }

  // Clear alerts of a specific severity
  void clearAlertsBySeverity(AlertSeverity severity)
  {
    alerts.erase(std::remove_if(alerts.begin(), alerts.end(),
                                [severity](const Alert &a)
                                { return a.severity == severity; }),
                 alerts.end());
  }

  // Acknowledge an alert
  void acknowledgeAlert(size_t index)
  {
    if (index < alerts.size())
    {
      alerts[index].acknowledged = true;
    }
  }

  // Get all alerts
  const std::vector<Alert> &getAlerts() const
  {
    return alerts;
  }

  // Get unacknowledged alerts
  std::vector<Alert> getUnacknowledgedAlerts() const
  {
    std::vector<Alert> unacked;
    for (const auto &alert : alerts)
    {
      if (!alert.acknowledged)
      {
        unacked.push_back(alert);
      }
    }
    return unacked;
  }

  // Get alerts by severity
  std::vector<Alert> getAlertsBySeverity(AlertSeverity severity) const
  {
    std::vector<Alert> filtered;
    for (const auto &alert : alerts)
    {
      if (alert.severity == severity)
      {
        filtered.push_back(alert);
      }
    }
    return filtered;
  }

  // Get alert counts by severity
  int getCriticalCount() const
  {
    return getAlertsBySeverity(AlertSeverity::CRITICAL).size();
  }

  int getWarningCount() const
  {
    return getAlertsBySeverity(AlertSeverity::WARNING).size();
  }

  int getInfoCount() const
  {
    return getAlertsBySeverity(AlertSeverity::INFO).size();
  }

  // Check if there are any critical alerts
  bool hasCriticalAlerts() const
  {
    return getCriticalCount() > 0;
  }

  // Check if there are any warnings
  bool hasWarnings() const
  {
    return getWarningCount() > 0;
  }

private:
  std::vector<Alert> alerts;
  size_t maxAlerts; // Maximum number of alerts to store
};

#endif // ALERTSYSTEM_H
