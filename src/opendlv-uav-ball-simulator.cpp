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
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <cmath>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ) {
        std::cerr << "You should include the cid to start communicate in OD4Session" << std::endl;
        return retCode;
    }

    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    float cur_y{0.0f};
    float dev{0.1f};
    while (od4.isRunning()) {
        // Sleep for 100 ms to not let the loop run to fast
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        opendlv::sim::Frame frame1;
        frame1.x(0.0f);
        frame1.y(0.0f);        
        frame1.z(1.0f);
        cluon::data::TimeStamp sampleTime;
        od4.send(frame1, sampleTime, 1);

        opendlv::sim::Frame frame2;
        if (cur_y >= 3.0f)
            dev = -0.1f;
        else if (cur_y <= -3.0f)
            dev = 0.1f;
        cur_y += dev;

        frame2.x(0.0f);
        frame2.y(cur_y);        
        frame2.z(1.0f);
        od4.send(frame2, sampleTime, 2);
    }

    retCode = 0;
    return retCode;
}