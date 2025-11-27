#include "GroundStation.h"

GroundStation::GroundStation(const glm::dvec3 &position)
    : position(position), connectedSatellite(nullptr)
{
}
