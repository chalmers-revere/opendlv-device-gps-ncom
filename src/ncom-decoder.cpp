/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "ncom-decoder.hpp"

#include <cmath>
#include <cstring>
#include <array>
#include <sstream>
#include <string>

std::pair<bool, NCOMDecoder::NCOMMessages> NCOMDecoder::decode(const std::string &data) noexcept {
    bool retVal{false};
    NCOMDecoder::NCOMMessages msg;

    constexpr std::size_t NCOM_PACKET_LENGTH{72};
    constexpr uint8_t NCOM_FIRST_BYTE{0xE7};
    if ( (NCOM_PACKET_LENGTH == data.size()) && (NCOM_FIRST_BYTE == static_cast<uint8_t>(data.at(0))) ) {
        std::stringstream buffer{data};

        // Decode latitude/longitude.
        {
            double latitude{0.0};
            double longitude{0.0};

            // Move to where latitude/longitude are encoded.
            constexpr uint32_t START_OF_LAT_LON{23};
            buffer.seekg(START_OF_LAT_LON);
            buffer.read(reinterpret_cast<char*>(&latitude), sizeof(double));
            buffer.read(reinterpret_cast<char*>(&longitude), sizeof(double));

            msg.position.latitude(latitude / M_PI * 180.0).longitude(longitude / M_PI * 180.0);
        }

        // Decode heading.
        {
            float northHeading{0.0f};

            // Move to where heading is encoded.
            constexpr uint32_t START_OF_HEADING{52};
            buffer.seekg(START_OF_HEADING);

            // Extract only three bytes from NCOM.
            std::array<char, 4> tmp{0, 0, 0, 0};
            buffer.read(tmp.data(), 3);
            uint32_t value{0};
            std::memcpy(&value, tmp.data(), 4);
            value = le32toh(value);
            northHeading = value * 1e-6f;

            // Normalize between -M_PI .. M_PI.
            while (northHeading < -M_PI) {
                northHeading += 2.0f * static_cast<float>(M_PI);
            }
            while (northHeading > M_PI) {
                northHeading -= 2.0f * static_cast<float>(M_PI);
            }

            msg.heading.northHeading(northHeading);
        }

        // Decode velocity
        {
            float northVelocity{0.0f};
            float eastVelocity{0.0f};
            float downVelocity{0.0f};

            std::array<char, 4> tmp{0, 0, 0, 0};
            uint32_t value{0};
            {
                // Move to where north velocity is encoded.
                constexpr uint32_t START_OF_NORTH_VELOCITY{43};
                buffer.seekg(START_OF_NORTH_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value);
                northVelocity = value * 1e-4f;
            }
            {
                // Move to where east velocity is encoded.
                constexpr uint32_t START_OF_EAST_VELOCITY{46};
                buffer.seekg(START_OF_EAST_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value);
                eastVelocity = value * 1e-4f;
            }
            {
                // Move to where down velocity is encoded.
                constexpr uint32_t START_OF_DOWN_VELOCITY{49};
                buffer.seekg(START_OF_DOWN_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value);
                downVelocity = value * -1e-4f;
            }

            msg.speed.groundSpeed(northVelocity + eastVelocity + downVelocity);
        }
        retVal = true;
    }
    return std::make_pair(retVal, msg);
}

