#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <string>
#include "nav_nmea.hpp"
#include "Obstacle.hpp"
#include "Waypoint.hpp"

namespace
{

std::string makeNmeaSentence(const std::string& body)
{
    uint8_t sum = 0;
    for (char c : body) {
        sum ^= static_cast<uint8_t>(c);
    }
    char hex[4];
    std::snprintf(hex, sizeof(hex), "%02X", static_cast<unsigned>(sum));
    return "$" + body + "*" + hex + "\r\n";
}

}  // namespace

TEST(NavNmeaTest, ChecksumValidationRejectsCorruptedAndMissingFrames)
{
    const std::string rmc_body = "GNRMC,123519.00,A,5127.60000,N,11242.00000,W,010.0,090.0,091126,,,A";
    const std::string valid = makeNmeaSentence(rmc_body);
    EXPECT_TRUE(nav_nmea::verifyNmeaChecksum(valid));

    const std::string bad_hex = "$" + rmc_body + "*00\r\n";
    EXPECT_FALSE(nav_nmea::verifyNmeaChecksum(bad_hex));
    EXPECT_FALSE(nav_nmea::verifyNmeaChecksum("$" + rmc_body + "\r\n"));
    EXPECT_FALSE(nav_nmea::verifyNmeaChecksum("no_dollar*00"));

    // Only valid sentence is returned even when followed by a corrupted sentence in buffer
    const std::string buf = valid + "$" + rmc_body + "*FF\r\n";
    auto fields = nav_nmea::nmeaFields(buf, "RMC", true);
    ASSERT_FALSE(fields.empty());
    EXPECT_EQ(fields[2], "A");

    auto only_bad = nav_nmea::nmeaFields(bad_hex, "RMC", true);
    EXPECT_TRUE(only_bad.empty());
}

TEST(NavNmeaTest, GgaQualityFiltersDeadReckoningAndNoFix)
{
    EXPECT_FALSE(nav_nmea::isAcceptableGgaQuality(0));  // invalid / no fix
    EXPECT_TRUE(nav_nmea::isAcceptableGgaQuality(1));   // GPS
    EXPECT_TRUE(nav_nmea::isAcceptableGgaQuality(2));   // DGPS
    EXPECT_TRUE(nav_nmea::isAcceptableGgaQuality(4));   // RTK fixed
    EXPECT_TRUE(nav_nmea::isAcceptableGgaQuality(5));   // RTK float
    EXPECT_FALSE(nav_nmea::isAcceptableGgaQuality(6));  // dead reckoning
    EXPECT_FALSE(nav_nmea::isAcceptableGgaQuality(8));  // simulated
}

TEST(NavNmeaTest, ResetHomeRebasesLocalCoordinatesCleanly)
{
    constexpr int kHomePassword = 741021;
    Waypoint home1(-112.70, 51.46, 0.0, true);
    ASSERT_TRUE(home1.setHome(&home1, kHomePassword));

    Obstacle obs(10.0, 20.0, 5);
    EXPECT_NEAR(obs.getX(), 10.0, 1e-3);
    EXPECT_NEAR(obs.getY(), 20.0, 1e-3);

    // Move home to roughly obs's geodetic location and rebase
    auto obs_geo = Waypoint::local_To_GeodeticApprox(10.0, 20.0);
    Waypoint home2(obs_geo[1], obs_geo[0], 0.0, true);
    ASSERT_TRUE(home2.setHome(&home2, kHomePassword));
    obs.rebaseToHome();
    EXPECT_NEAR(obs.getX(), 0.0, 0.1);
    EXPECT_NEAR(obs.getY(), 0.0, 0.1);
}
