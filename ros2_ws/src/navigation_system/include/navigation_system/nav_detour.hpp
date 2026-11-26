#pragma once

#include <array>
#include <cmath>

namespace nav_detour
{

/**
 * Unit vector from the point of interest (closest point of the track to the obstacle) towards the
 * obstacle centre. When the track passes through the centre that direction is undefined, so the
 * track's left normal is used instead. Returns {0, 0} if neither is defined.
 */
inline std::array<double, 2> detourDirection(
    double centreX, double centreY, double poiX, double poiY, double trackDx, double trackDy)
{
    const double dx = centreX - poiX;
    const double dy = centreY - poiY;
    const double length = std::hypot(dx, dy);
    if (length >= 1e-9) {
        return {dx / length, dy / length};
    }
    const double trackLength = std::hypot(trackDx, trackDy);
    if (trackLength >= 1e-9) {
        return {-trackDy / trackLength, trackDx / trackLength};
    }
    return {0.0, 0.0};
}

/**
 * Candidate detour waypoint `distance` metres from the obstacle CENTRE along `direction`:
 * side = -1 is the near side (the side the track passes on), +1 the far side.
 * Anchoring on the centre (not the point of interest) keeps the waypoint exactly `distance`
 * from the centre however far off-centre the track passes.
 */
inline std::array<double, 2> detourPoint(
    double centreX, double centreY, const std::array<double, 2>& direction, double distance, int side)
{
    return {centreX + side * distance * direction[0], centreY + side * distance * direction[1]};
}

}  // namespace nav_detour
