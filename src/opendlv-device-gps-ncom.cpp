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
#include "opendlv-standard-message-set.hpp"

#include "ncom-decoder.hpp"

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("ncom_port")) || (0 == commandlineArguments.count("cid")) ) {
        std::cerr << argv[0] << " decodes latitude/longitude/heading from an OXTS GPS/INSS unit in NCOM format and publishes it to a running OpenDaVINCI session using the OpenDLV Standard Message Set." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " [--ncom_ip=<IPv4-address>] --ncom_port=<port> --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple OxTS units>] [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --ncom_ip=0.0.0.0 --ncom_port=3000 --cid=111" << std::endl;
        retCode = 1;
    } else {
        const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
            [](auto){}
        };

        // Interface to OxTS unit providing data in NCOM format.
        const std::string NCOM_ADDRESS((commandlineArguments.count("ncom_ip") == 0) ? "0.0.0.0" : commandlineArguments["ncom_ip"]);
        const uint32_t NCOM_PORT(std::stoi(commandlineArguments["ncom_port"]));
        NCOMDecoder ncomDecoder;
        cluon::UDPReceiver fromDevice(NCOM_ADDRESS, NCOM_PORT,
            [&od4Session = od4, &decoder = ncomDecoder, senderStamp = ID, VERBOSE](std::string &&d, std::string &&/*from*/, std::chrono::system_clock::time_point &&tp) noexcept {
            auto retVal = decoder.decode(d);
            if (retVal.first) {
                cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

                opendlv::proxy::GeodeticWgs84Reading msg1 = retVal.second.position;
                od4Session.send(msg1, sampleTime, senderStamp);

                opendlv::proxy::GeodeticHeadingReading msg2 = retVal.second.heading;
                od4Session.send(msg2, sampleTime, senderStamp);

                opendlv::proxy::GroundSpeedReading msg3 = retVal.second.speed;
                od4Session.send(msg3, sampleTime, senderStamp);

                opendlv::proxy::AltitudeReading msg4 = retVal.second.altitude;
                od4Session.send(msg4, sampleTime, senderStamp);

                opendlv::logic::sensation::Geolocation msg5 = retVal.second.geolocation;
                od4Session.send(msg5, sampleTime, senderStamp);

                // Print values on console.
                if (VERBOSE) {
                    std::stringstream buffer;
                    msg1.accept([](uint32_t, const std::string &, const std::string &) {},
                               [&buffer](uint32_t, std::string &&, std::string &&n, auto v) { buffer << n << " = " << v << '\n'; },
                               []() {});
                    std::cout << buffer.str() << std::endl;

                    std::stringstream buffer2;
                    msg2.accept([](uint32_t, const std::string &, const std::string &) {},
                               [&buffer2](uint32_t, std::string &&, std::string &&n, auto v) { buffer2 << n << " = " << v << '\n'; },
                               []() {});
                    std::cout << buffer2.str() << std::endl;

                    std::stringstream buffer3;
                    msg3.accept([](uint32_t, const std::string &, const std::string &) {},
                               [&buffer3](uint32_t, std::string &&, std::string &&n, auto v) { buffer3 << n << " = " << v << '\n'; },
                               []() {});
                    std::cout << buffer3.str() << std::endl;

                    std::stringstream buffer4;
                    msg4.accept([](uint32_t, const std::string &, const std::string &) {},
                               [&buffer4](uint32_t, std::string &&, std::string &&n, auto v) { buffer4 << n << " = " << v << '\n'; },
                               []() {});
                    std::cout << buffer4.str() << std::endl;

                    std::stringstream buffer5;
                    msg5.accept([](uint32_t, const std::string &, const std::string &) {},
                               [&buffer5](uint32_t, std::string &&, std::string &&n, auto v) { buffer5 << n << " = " << v << '\n'; },
                               []() {});
                    std::cout << buffer5.str() << std::endl;
                }
            }
        });

        // Just sleep as this microservice is data driven.
        using namespace std::literals::chrono_literals;
        while (od4.isRunning()) {
            std::this_thread::sleep_for(1s);
        }
    }
    return retCode;
}
