#pragma once

#include <cmath>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

namespace nav_nmea
{

/** Convert NMEA ddmm.mmmm coordinate notation into decimal degrees. */
inline double nmeaToDegrees(double ddmm)
{
    double degrees = std::floor(ddmm / 100.0);
    return degrees + (ddmm - degrees * 100.0) / 60.0;
}

/** Validate an NMEA 0183 XOR checksum (*hh). Returns false when absent, malformed, or mismatched. */
inline bool verifyNmeaChecksum(const std::string& sentence)
{
    const size_t dollar = sentence.find('$');
    const size_t star = sentence.find('*', dollar == std::string::npos ? 0 : dollar + 1);
    if (dollar == std::string::npos || star == std::string::npos || star + 2 >= sentence.size()) {
        return false;
    }

    const unsigned char hi = static_cast<unsigned char>(sentence[star + 1]);
    const unsigned char lo = static_cast<unsigned char>(sentence[star + 2]);
    if (!std::isxdigit(hi) || !std::isxdigit(lo)) {
        return false;
    }

    uint8_t computed = 0;
    for (size_t i = dollar + 1; i < star; ++i) {
        computed ^= static_cast<uint8_t>(sentence[i]);
    }

    const int expected = std::stoi(sentence.substr(star + 1, 2), nullptr, 16);
    return static_cast<int>(computed) == expected;
}

/** Accept GPS (1), DGPS (2), RTK Fixed (4), and RTK Float (5); reject no-fix (0) and DR (6). */
inline bool isAcceptableGgaQuality(int quality)
{
    return quality == 1 || quality == 2 || quality == 4 || quality == 5;
}

/**
 * Extract comma-delimited fields of the newest complete NMEA sentence matching `type`
 * (e.g., "RMC", "GGA", "VTG"). Rejects frames failing XOR checksum when `require_checksum` is true.
 */
inline std::vector<std::string> nmeaFields(
    const std::string& buffer, const char* type, bool require_checksum = true)
{
    std::vector<std::string> fields;
    size_t pos = buffer.rfind(type);
    while (pos != std::string::npos) {
        const size_t begin = buffer.rfind('$', pos);
        const size_t end = buffer.find_first_of("*\r\n", pos);
        if (begin != std::string::npos && end != std::string::npos && pos - begin == 3) {
            bool checksum_ok = true;
            if (require_checksum) {
                if (buffer[end] != '*' || end + 2 >= buffer.size()) {
                    checksum_ok = false;
                } else {
                    checksum_ok = verifyNmeaChecksum(buffer.substr(begin, (end + 3) - begin));
                }
            }
            if (checksum_ok) {
                const std::string sentence = buffer.substr(begin, end - begin);
                size_t from = 0;
                while (true) {
                    const size_t comma = sentence.find(',', from);
                    fields.push_back(sentence.substr(
                        from, comma == std::string::npos ? std::string::npos : comma - from));
                    if (comma == std::string::npos) {
                        break;
                    }
                    from = comma + 1;
                }
                return fields;
            }
        }
        if (pos == 0) {
            break;
        }
        pos = buffer.rfind(type, pos - 1);
    }
    return fields;
}

}  // namespace nav_nmea
