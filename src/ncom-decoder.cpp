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
#include <ctime>
#include <array>
#include <iostream>
#include <sstream>
#include <string>

std::pair<bool, NCOMDecoder::NCOMMessages> NCOMDecoder::decode(const std::string &data) noexcept {
    bool retVal{false};
    NCOMDecoder::NCOMMessages msg;

    const constexpr std::size_t NCOM_PACKET_LENGTH{72};
    const constexpr uint8_t NCOM_FIRST_BYTE{0xE7};
    if ( (NCOM_PACKET_LENGTH == data.size()) && (NCOM_FIRST_BYTE == static_cast<uint8_t>(data.at(0))) ) {
        std::stringstream buffer{data};

        // Time stamping.
        {
            // Test for channel 0 that has complete time stamp.
            {
                const constexpr uint32_t START_OF_CHANNEL{62};
                buffer.seekg(START_OF_CHANNEL);
                char channel{0};
                buffer.read(&channel, sizeof(char));
                if (0 == channel) {
                    const constexpr uint32_t START_OF_GPSMINUTES{63};
                    buffer.seekg(START_OF_GPSMINUTES);
                    buffer.read(reinterpret_cast<char*>(&m_gpsMinutes), sizeof(uint32_t));
                    m_gpsMinutes = le32toh(m_gpsMinutes);
                }
            }

            // When we have a GPS minute from a previous run:
            if (0 < m_gpsMinutes) {
                // Move to where the timestamp is encoded as we have a valid GPS
                // minute time stamp either from the current cycle or from a previous one.
                const constexpr uint32_t START_OF_TIMESTAMP{1};
                uint16_t millisecondsIntoCurrentGPSMinute{0};
                buffer.seekg(START_OF_TIMESTAMP);
                buffer.read(reinterpret_cast<char*>(&(millisecondsIntoCurrentGPSMinute)), sizeof(uint16_t));
                millisecondsIntoCurrentGPSMinute = le16toh(millisecondsIntoCurrentGPSMinute);

                const constexpr int32_t GPS_EPOCH_OFFSET{315964800};
                const constexpr int32_t GPS_LEAP_SECONDS{18};
                msg.sampleTime.seconds(GPS_EPOCH_OFFSET - GPS_LEAP_SECONDS + 60*static_cast<int32_t>(m_gpsMinutes) + static_cast<int32_t>(millisecondsIntoCurrentGPSMinute)/1000).microseconds((millisecondsIntoCurrentGPSMinute%1000)*1000);
            }
        }

        // Decode acceleration.
        {
            float accelerationX{0.0f};
            float accelerationY{0.0f};
            float accelerationZ{0.0f};

            std::array<char, 4> tmp{0, 0, 0, 0};
            int32_t value{0};
            {
                // Move to where acceleration X is encoded.
                const constexpr uint32_t START_OF_ACCELERATIONX{3};
                buffer.seekg(START_OF_ACCELERATIONX);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                accelerationX = value * 1e-4f;
            }
            {
                // Move to where acceleration Y is encoded.
                const constexpr uint32_t START_OF_ACCELERATIONY{6};
                buffer.seekg(START_OF_ACCELERATIONY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                accelerationY = value * 1e-4f;
            }
            {
                // Move to where acceleration Z is encoded.
                const constexpr uint32_t START_OF_ACCELERATIONZ{9};
                buffer.seekg(START_OF_ACCELERATIONZ);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                accelerationZ = value * 1e-4f;
            }

            msg.acceleration.accelerationX(accelerationX)
                            .accelerationY(accelerationY)
                            .accelerationZ(accelerationZ);
            retVal = true;
        }

        // Decode angular velocity.
        {
            float angularRateX{0.0f};
            float angularRateY{0.0f};
            float angularRateZ{0.0f};

            std::array<char, 4> tmp{0, 0, 0, 0};
            int32_t value{0};
            {
                // Move to where angular rate X is encoded.
                const constexpr uint32_t START_OF_ANGULARRATEX{12};
                buffer.seekg(START_OF_ANGULARRATEX);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                angularRateX = value * 1e-5f;
            }
            {
                // Move to where angular rate Y is encoded.
                const constexpr uint32_t START_OF_ANGULARRATEY{15};
                buffer.seekg(START_OF_ANGULARRATEY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                angularRateY = value * 1e-5f;
            }
            {
                // Move to where angular rate Z is encoded.
                const constexpr uint32_t START_OF_ANGULARRATEZ{18};
                buffer.seekg(START_OF_ANGULARRATEZ);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                angularRateZ = value * 1e-5f;
            }

            msg.angularVelocity.angularVelocityX(angularRateX)
                               .angularVelocityY(angularRateY)
                               .angularVelocityZ(angularRateZ);
            retVal &= true;
        }

        // Decode latitude/longitude.
        {
            double latitude{0.0};
            double longitude{0.0};

            // Move to where latitude/longitude are encoded.
            const constexpr uint32_t START_OF_LAT_LON{23};
            buffer.seekg(START_OF_LAT_LON);
            buffer.read(reinterpret_cast<char*>(&latitude), sizeof(double));
            buffer.read(reinterpret_cast<char*>(&longitude), sizeof(double));

            msg.position.latitude(latitude / M_PI * 180.0).longitude(longitude / M_PI * 180.0);

            // Update Geolocation.
            msg.geolocation.latitude(msg.position.latitude());
            msg.geolocation.longitude(msg.position.longitude());

            const auto p{(std::fabs(latitude) * std::fabs(longitude))};
            retVal &= (0 < p) && (p < 90.0*180.0);
        }

        // Decode altitude.
        {
            float altitude{0.0f};

            // Move to where altitude is encoded.
            const constexpr uint32_t START_OF_ALT{39};
            buffer.seekg(START_OF_ALT);
            buffer.read(reinterpret_cast<char*>(&altitude), sizeof(float));

            msg.altitude.altitude(altitude);

            // Update Geolocation.
            msg.geolocation.altitude(msg.altitude.altitude());
            retVal &= true;
        }

        // Decode velocity
        {
            float northVelocity{0.0f};
            float eastVelocity{0.0f};
            float downVelocity{0.0f};

            std::array<char, 4> tmp{0, 0, 0, 0};
            int32_t value{0};
            {
                // Move to where north velocity is encoded.
                const constexpr uint32_t START_OF_NORTH_VELOCITY{43};
                buffer.seekg(START_OF_NORTH_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                northVelocity = value * 1e-4f;
            }
            {
                // Move to where east velocity is encoded.
                const constexpr uint32_t START_OF_EAST_VELOCITY{46};
                buffer.seekg(START_OF_EAST_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                eastVelocity = value * -1e-4f;
            }
            {
                // Move to where down velocity is encoded.
                const constexpr uint32_t START_OF_DOWN_VELOCITY{49};
                buffer.seekg(START_OF_DOWN_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                downVelocity = value * -1e-4f;
            }

            msg.speed.groundSpeed(northVelocity + eastVelocity + downVelocity);
            retVal &= true;
        }

        // Added by Yue Kang
        // Integrate linear and angular velocities into equilibrioception
        {
            float angularRateX{0.0f};
            float angularRateY{0.0f};
            float angularRateZ{0.0f};

            std::array<char, 4> tmp{0, 0, 0, 0};
            int32_t value{0};
            {
                // Move to where angular rate X is encoded.
                const constexpr uint32_t START_OF_ANGULARRATEX{12};
                buffer.seekg(START_OF_ANGULARRATEX);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                angularRateX = value * 1e-5f;
            }
            {
                // Move to where angular rate Y is encoded.
                const constexpr uint32_t START_OF_ANGULARRATEY{15};
                buffer.seekg(START_OF_ANGULARRATEY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                angularRateY = value * 1e-5f;
            }
            {
                // Move to where angular rate Z is encoded.
                const constexpr uint32_t START_OF_ANGULARRATEZ{18};
                buffer.seekg(START_OF_ANGULARRATEZ);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                angularRateZ = value * 1e-5f;
            }

            float northVelocity{0.0f};
            float eastVelocity{0.0f};
            float downVelocity{0.0f};

            tmp.fill(0);
            value = 0;
            {
                // Move to where north velocity is encoded.
                const constexpr uint32_t START_OF_NORTH_VELOCITY{43};
                buffer.seekg(START_OF_NORTH_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                northVelocity = value * 1e-4f;
            }
            {
                // Move to where east velocity is encoded.
                const constexpr uint32_t START_OF_EAST_VELOCITY{46};
                buffer.seekg(START_OF_EAST_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                eastVelocity = value * -1e-4f;
            }
            {
                // Move to where down velocity is encoded.
                const constexpr uint32_t START_OF_DOWN_VELOCITY{49};
                buffer.seekg(START_OF_DOWN_VELOCITY);

                // Extract only three bytes from NCOM.
                buffer.read(tmp.data(), 3);
                std::memcpy(&value, tmp.data(), 4);
                value = le32toh(value) & 0xFFFFFF;
                if ((value & 0x800000) == 0x800000) {
                    value = -1 * ((~value & 0xFFFFFF) + 1);
                }
                downVelocity = value * -1e-4f;
            }

            // msg.speed.groundSpeed(northVelocity + eastVelocity + downVelocity);

            // msg.angularVelocity.angularVelocityX(angularRateX)
            //                    .angularVelocityY(angularRateY)
            //                    .angularVelocityZ(angularRateZ);

            // The following data is currently NOT CONVERTED to local frame
            // Currently the conversion from the receiver end is necessary
            msg.equilibrioception.vx(northVelocity)
                                 .vy(eastVelocity)
                                 .vz(downVelocity)
                                 .rollRate(angularRateX)
                                 .pitchRate(angularRateY)
                                 .yawRate(angularRateZ);
            retVal &= true;
        }

        // Decode heading.
        {
            float heading{0.0f};

            // Move to where heading is encoded.
            const constexpr uint32_t START_OF_HEADING{52};
            buffer.seekg(START_OF_HEADING);

            // Extract only three bytes from NCOM.
            std::array<char, 4> tmp{0, 0, 0, 0};
            buffer.read(tmp.data(), 3);
            uint32_t value{0};
            std::memcpy(&value, tmp.data(), 4);
            value = le32toh(value);
            heading = value * 1e-6f;

            // Normalize between -M_PI .. M_PI.
            while (heading < -M_PI) {
                heading += 2.0f * static_cast<float>(M_PI);
            }
            while (heading > M_PI) {
                heading -= 2.0f * static_cast<float>(M_PI);
            }

            msg.heading.northHeading(heading);

            // Update Geolocation.
            msg.geolocation.heading(msg.heading.northHeading());
            retVal &= true;
        }

        // Decode pitch.
        {
            float pitch{0.0f};

            // Move to where pitch is encoded.
            const constexpr uint32_t START_OF_PITCH{55};
            buffer.seekg(START_OF_PITCH);

            // Extract only three bytes from NCOM.
            std::array<char, 4> tmp{0, 0, 0, 0};
            buffer.read(tmp.data(), 3);
            uint32_t value{0};
            std::memcpy(&value, tmp.data(), 4);
            value = le32toh(value);
            pitch = value * 1e-6f;

            // Normalize between -M_PI/2.0 .. M_PI/2.0.
            while (pitch < -static_cast<float>(M_PI)/2.0f) {
                pitch += static_cast<float>(M_PI);
            }
            while (pitch > static_cast<float>(M_PI)/2.0f) {
                pitch -= static_cast<float>(M_PI);
            }

            msg.pitch = pitch;
            retVal &= true;
        }

        // Decode roll.
        {
            float roll{0.0f};

            // Move to where roll is encoded.
            const constexpr uint32_t START_OF_ROLL{58};
            buffer.seekg(START_OF_ROLL);

            // Extract only three bytes from NCOM.
            std::array<char, 4> tmp{0, 0, 0, 0};
            buffer.read(tmp.data(), 3);
            uint32_t value{0};
            std::memcpy(&value, tmp.data(), 4);
            value = le32toh(value);
            roll = value * 1e-6f;

            // Normalize between -M_PI .. M_PI.
            while (roll < -M_PI) {
                roll += 2.0f * static_cast<float>(M_PI);
            }
            while (roll > M_PI) {
                roll -= 2.0f * static_cast<float>(M_PI);
            }

            msg.roll = roll;
            retVal &= true;
        }
    }
    return std::make_pair(retVal, msg);
}

