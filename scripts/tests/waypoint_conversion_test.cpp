// Numeric check of the Waypoint coordinate conversions (geodetic <-> ECEF <-> local East/North).
// No ROS needed. From the repository root:
//   g++ -std=c++17 -Iros2_ws/src/navigation_system/include/navigation_system ros2_ws/src/navigation_system/src/Waypoint.cpp scripts/tests/waypoint_conversion_test.cpp -o waypoint_test
//   ./waypoint_test
#include <cmath>
#include <cstdio>
#include "Waypoint.hpp"

static int failures = 0;
static void expectNear(const char* what, double got, double want, double tol) {
    bool ok = std::fabs(got - want) <= tol;
    std::printf("%-44s got %14.7f want %14.7f %s\n", what, got, want, ok ? "ok" : "FAIL");
    if (!ok) failures++;
}

int main() {
    const double homeLat = 51.46, homeLon = -112.70, homeAlt = 792.0;
    Waypoint* home = new Waypoint(homeLon, homeLat, homeAlt, true);  // constructor order: (lon, lat, alt)
    if (!home->setHome(home, 741021)) { std::puts("could not set home"); return 2; }

    // A point 100 m east / 200 m north of home, built from local coordinates.
    Waypoint local(100.0, 200.0);
    const double mPerDegLat = 111132.0;   // approximate metres per degree of latitude near 51.5 N
    const double mPerDegLon = 111320.0 * std::cos(homeLat * 3.14159265358979323846 / 180.0);
    expectNear("local->geodetic latitude",  local.getLatitude(),  homeLat + 200.0 / mPerDegLat, 2e-5);
    expectNear("local->geodetic longitude", local.getLongitude(), homeLon + 100.0 / mPerDegLon, 2e-5);

    // Round trip: geodetic -> local -> geodetic.
    Waypoint g(homeLon + 0.001, homeLat + 0.001, homeAlt, true);
    expectNear("geodetic->local x (east, m)",  g.getX(), 0.001 * mPerDegLon, 0.5);
    expectNear("geodetic->local y (north, m)", g.getY(), 0.001 * mPerDegLat, 0.5);

    // Static helper returns degrees {lat, lon}.
    auto ll = Waypoint::local_To_GeodeticApprox(100.0, 200.0);
    expectNear("static helper latitude",  ll[0], local.getLatitude(),  1e-9);
    expectNear("static helper longitude", ll[1], local.getLongitude(), 1e-9);

    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
