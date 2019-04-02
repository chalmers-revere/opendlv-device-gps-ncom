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

#ifndef NCOM_DECODER
#define NCOM_DECODER

#include "opendlv-standard-message-set.hpp"

#include <string>
#include <utility>

class NCOMDecoder {
   public:
    class NCOMMessages {
       public:
        cluon::data::TimeStamp sampleTime{};
        opendlv::proxy::AccelerationReading acceleration{};
        opendlv::proxy::AngularVelocityReading angularVelocity{};
        opendlv::proxy::GeodeticWgs84Reading position{};
        opendlv::proxy::GeodeticHeadingReading heading{};
        opendlv::proxy::GroundSpeedReading speed{};
        opendlv::proxy::AltitudeReading altitude{};
        opendlv::logic::sensation::Geolocation geolocation{};
        opendlv::logic::sensation::Equilibrioception equilibrioception{};
        float pitch{0.0f};
        float roll{0.0f};
    };

   private:
    NCOMDecoder(const NCOMDecoder &) = delete;
    NCOMDecoder(NCOMDecoder &&)      = delete;
    NCOMDecoder &operator=(const NCOMDecoder &) = delete;
    NCOMDecoder &operator=(NCOMDecoder &&) = delete;

   public:
    NCOMDecoder() = default;
    ~NCOMDecoder() = default;

   public:
    std::pair<bool, NCOMMessages> decode(const std::string &data) noexcept;

   private:
    uint32_t m_gpsMinutes{0};
};

#endif

