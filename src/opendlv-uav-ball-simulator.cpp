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

    struct cfPos {
        float x;
        float y;
    };
    std::mutex stateMutex;
    cfPos cur_pos{0.0f, 0.0f};
    int FRAME_ID = 0;
    auto onFrame{[&FRAME_ID, &cur_pos, &stateMutex](cluon::data::Envelope &&envelope)
    {
        uint32_t const senderStamp = envelope.senderStamp();
        if (FRAME_ID == senderStamp) {
            auto frame = cluon::extractMessage<opendlv::sim::Frame>(std::move(envelope));
            std::lock_guard<std::mutex> lck(stateMutex);
            cur_pos.x = frame.x();
            cur_pos.y = frame.y();
        }
    }};
    od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);

    float dist_obs{-1.0f};
    std::mutex distMutex;
    auto onDistRead = [&distMutex, &dist_obs](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::logic::action::PreviewPoint pPtmessage = cluon::extractMessage<opendlv::logic::action::PreviewPoint>(std::move(env));
        
        // Store distance readings.
        std::lock_guard<std::mutex> lck(distMutex);
        if ( senderStamp == 1 ){
            dist_obs = pPtmessage.distance();
        }
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::logic::action::PreviewPoint::ID(), onDistRead);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::cout <<" Start ball simulation..." << std::endl;
    float cur_x{0.0f};
    float dev{0.1f};
    int nTimer = 0;
    float targetx{1.0f};
    float targety{-1.0f};
    int16_t nTargetFoundTimer{0};

    while (od4.isRunning()) {
        // Sleep for 100 ms to not let the loop run to fast
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        opendlv::sim::Frame frame1;
        opendlv::sim::Frame frame2;
        opendlv::logic::sensation::TargetFoundState targetFoundState;

        float dist = std::sqrt(std::pow(cur_pos.x - targetx,2) + std::pow(cur_pos.y - targety,2));
        // std::cout <<" Current distance: " << dist << std::endl;
        if ( dist <= 0.3f ){
            if ( targetx == 1.0f && targety == -1.0f ){
                targetx = -0.7f;
            }
            else if ( targetx == -0.7f && targety == -1.0f ){
                targetx = 1.0f;
                targety = 0.0f;
            }
            else{
                targetx = 1.0f;
                targety = -1.0f;
            }
            nTargetFoundTimer += 1;
        }
        frame1.x(targetx);
        frame1.y(targety);        
        frame1.z(1.0f);

        if (cur_x >= 1.25f)
            dev = -0.1f;
        else if (cur_x <= -0.75f)
            dev = 0.1f;

        if ( dist_obs > 0.1f )
            cur_x += dev;    

        if ( nTimer <= 3000 ){
            frame2.x(cur_x);
            frame2.y(0.0f); 
            frame2.z(1.0f);
        }
        else if ( nTimer <= 6000 ) {
            frame2.x(-5.0f);
            frame2.y(-5.0f); 
            frame2.z(1.0f);
        }
        else if ( nTimer <= 9000 ) {
            frame2.x(0.0f); 
            frame2.y(cur_x);
            frame2.z(1.0f);
        }
        else{
            nTimer = 0;
        }

        targetFoundState.target_found_count(nTargetFoundTimer);
        cluon::data::TimeStamp sampleTime;
        od4.send(frame1, sampleTime, 1);
        od4.send(frame2, sampleTime, 2);
        od4.send(targetFoundState, sampleTime, 0);
        nTimer += 1;

        // opendlv::sim::Frame frame2;
        // if (cur_x >= 3.0f)
        //     dev = -0.1f;
        // else if (cur_x <= -3.0f)
        //     dev = 0.1f;
        // cur_x += dev;
        // frame2.x(0.2f);
        // frame2.y(cur_x); 
        // frame2.z(1.0f);
        // cluon::data::TimeStamp sampleTime;
    }

    retCode = 0;
    return retCode;
}