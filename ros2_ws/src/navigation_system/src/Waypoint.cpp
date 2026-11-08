#ifndef WAYPOINT_CPP
#define WAYPOINT_CPP

#include <cmath>
#include "Waypoint.hpp"

// // Definitions for static members
//  Waypoint::a = 6378137.0;                // Semi-major axis (meters)
//  Waypoint::f = 1.0 / 298.257223565;      // Flattening
//  Waypoint::e2 = f * (2.0 - f);           // Eccentricity squared
//  Waypoint::pi = 3.14159265358979323846;  // pi

//  Waypoint::home_x = 0.0;          // local coordinates of the home base
//  Waypoint::home_y = 0.0;          // local coordinates of the home base

//  Waypoint::home_ECEF_x = 0.0;
//  Waypoint::home_ECEF_y = 0.0;
//  Waypoint::home_ECEF_z = 0.0;

//  Waypoint::home_latitude = 0.0;
//  Waypoint::home_longitude = 0.0;
//  Waypoint::home_altitude = 792.18; //altitude of Dumheller airport (acording to CFS from August 2023)


// constructor for when using local coordinates
Waypoint::Waypoint(double newX, double newY) : x(newX), y(newY) {
    local_To_EarthCentred();
    EarthCentred_To_GeodeticApprox();
}

// constructor for when using geodetic or Earth Centred coordinates
Waypoint::Waypoint(double coordinateA, double coordinateB, double coordinateC, bool geodeticCoordinate) {
    if (geodeticCoordinate){
        longitude = coordinateA;
        latitude = coordinateB;
        altitude = coordinateC;

        geodetic_To_EarthCentred();
        EarthCentred_To_Local();
    }

    else {
        ECEF_x = coordinateA;
        ECEF_y = coordinateB;
        ECEF_z = coordinateC;

        EarthCentred_To_Local();
        EarthCentred_To_GeodeticApprox();
    }
}

// Conversion Methods

void Waypoint::geodetic_To_EarthCentred() {
    double latitudeRadians = latitude * pi / 180.0;
    double longitudeRadians = longitude * pi / 180.0;

    double N = a / std::sqrt(
        1.0 - e2 *
        std::sin(latitudeRadians) *
        std::sin(latitudeRadians)
    );

    ECEF_x =
        (N + altitude) *
        std::cos(latitudeRadians) *
        std::cos(longitudeRadians);

    ECEF_y =
        (N + altitude) *
        std::cos(latitudeRadians) *
        std::sin(longitudeRadians);

    ECEF_z =
        (N * (1.0 - e2) + altitude) *
        std::sin(latitudeRadians);
}

void Waypoint::EarthCentred_To_Local() {
    // home_latitude / home_longitude are stored in degrees; the ENU rotation needs radians
    const double lat0 = home_latitude * pi / 180.0;
    const double lon0 = home_longitude * pi / 180.0;
    double dX = ECEF_x - home_ECEF_x;
    double dY = ECEF_y - home_ECEF_y;
    double dZ = ECEF_z - home_ECEF_z;
    
    x = -sin(lon0) * dX 
        +cos(lon0) * dY;

    y = -sin(lat0) * cos(lon0) * dX 
        -sin(lat0) * sin(lon0) * dY 
        +cos(lat0) * dZ;
}

void Waypoint::local_To_EarthCentred() {
    // home_latitude / home_longitude are stored in degrees; the ENU rotation needs radians
    const double lat0 = home_latitude * pi / 180.0;
    const double lon0 = home_longitude * pi / 180.0;
    double dX =
        -sin(lon0) * x
        -sin(lat0) * cos(lon0) * y;

    double dY =
            cos(lon0) * x
        -sin(lat0) * sin(lon0) * y;

    double dZ =
        cos(lat0) * y;

    ECEF_x = home_ECEF_x + dX;
    ECEF_y = home_ECEF_y + dY;
    ECEF_z = home_ECEF_z + dZ;
}

std::array<double,3> Waypoint::local_To_EarthCentred(double x, double y){
    // home_latitude / home_longitude are stored in degrees; the ENU rotation needs radians
    const double lat0 = home_latitude * pi / 180.0;
    const double lon0 = home_longitude * pi / 180.0;
    double dX =
        -sin(lon0) * x
        -sin(lat0) * cos(lon0) * y;

    double dY =
            cos(lon0) * x
        -sin(lat0) * sin(lon0) * y;

    double dZ =
        cos(lat0) * y;

    return {home_ECEF_x + dX, home_ECEF_y + dY, home_ECEF_z + dZ};
}

void Waypoint::EarthCentred_To_GeodeticApprox() {
    double p = sqrt(ECEF_x * ECEF_x + ECEF_y * ECEF_y);
    if (p < 1e-9 && std::abs(ECEF_z) < 1e-9) {
        latitude = 0.0;
        longitude = 0.0;
        return;
    }

    double lonRad = atan2(ECEF_y, ECEF_x);
    double latRad = atan2(ECEF_z, p * (1.0 - e2));
    double h = 0.0;

    for (int i = 0; i < 5; i++) {
        double N = calcPrimeVerticalRadius(latRad);
        double denom = N + h;
        if (std::abs(denom) < 1e-9) break;
        h = p / std::cos(latRad) - N;
        latRad = atan2(ECEF_z, p * (1.0 - e2 * N / denom));
    }

    latitude = latRad * 180.0 / pi;
    longitude = lonRad * 180.0 / pi;
    altitude = h;
}

std::array<double,2> Waypoint::local_To_GeodeticApprox(double x, double y){
    auto ECEF_point = local_To_EarthCentred(x, y);

    double p = sqrt(ECEF_point[0] * ECEF_point[0] + ECEF_point[1] * ECEF_point[1]);
    if (p < 1e-9 && std::abs(ECEF_point[2]) < 1e-9) {
        return {0, 0};
    }

    double lonRad = atan2(ECEF_point[1], ECEF_point[0]);
    double latRad = atan2(ECEF_point[2], p * (1.0 - e2));
    double h = 0.0;

    for (int i = 0; i < 5; i++) {
        double N = calcPrimeVerticalRadius(latRad);
        double denom = N + h;
        if (std::abs(denom) < 1e-9) break;
        h = p / std::cos(latRad) - N;
        latRad = atan2(ECEF_point[2], p * (1.0 - e2 * N / denom));
    }

    // returned as {latitude, longitude} in degrees, matching Waypoint::latitude / longitude
    return {latRad * 180.0 / pi, lonRad * 180.0 / pi};
}

#endif // WAYPOINT_CPP
