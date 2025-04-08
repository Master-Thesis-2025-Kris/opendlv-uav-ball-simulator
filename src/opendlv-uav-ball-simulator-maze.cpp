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
    float chpadx{0.0f};
    if ( (0 == commandlineArguments.count("chpadx")) ) {
        std::cerr << "You should include the chpadx to start..." << std::endl;
        return retCode;
    }
    else{
        chpadx = static_cast<float>(std::stof(commandlineArguments["chpadx"]));
    }
    float chpady{0.0f};
    if ( (0 == commandlineArguments.count("chpady")) ) {
        std::cerr << "You should include the chpady to start..." << std::endl;
        return retCode;
    }
    else{
        chpady = static_cast<float>(std::stof(commandlineArguments["chpady"]));
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
    // For maze
    float targetx{-0.65f};
    float targety{-0.0f};
    float targetx_1{1.25f};
    float targety_1{-1.0f};
    int16_t nTargetFoundTimer{0};
    int16_t isChpadFound{0};

    bool isCloseToWall = false;
    bool isCloseToBall = false;
    auto closeWallStartTime = std::chrono::high_resolution_clock::now();
    auto closeWallEndTime = std::chrono::high_resolution_clock::now();
    auto closeBallStartTime = std::chrono::high_resolution_clock::now();
    auto closeBallEndTime = std::chrono::high_resolution_clock::now();

    while (od4.isRunning()) {
        // Sleep for 100 ms to not let the loop run to fast
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        opendlv::sim::Frame frame1;
        opendlv::sim::Frame frame2;
        opendlv::sim::Frame frame3;
        opendlv::logic::sensation::TargetFoundState targetFoundState;

        // Check current states
        // if ( (cur_pos.x >= 1.45 || cur_pos.x <= -0.95 || cur_pos.y >= 0.45 || cur_pos.y <= -1.45) ){
        //     if ( isCloseToWall == false ){
        //         std::cout << "Too close to the wall!!" << std::endl;
        //         auto closeWallStartTime = std::chrono::high_resolution_clock::now();
        //         isCloseToWall = true;
        //     }
        // }
        // else if ( isCloseToWall ){
        //     closeWallEndTime = std::chrono::high_resolution_clock::now();
        //     const std::chrono::duration<double> elapsed = closeWallEndTime - closeWallStartTime;
        //     auto start_time_t = std::chrono::system_clock::to_time_t(
        //         std::chrono::time_point_cast<std::chrono::system_clock::duration>(closeWallStartTime)
        //     );
        //     auto end_time_t = std::chrono::system_clock::to_time_t(
        //         std::chrono::time_point_cast<std::chrono::system_clock::duration>(closeWallEndTime)
        //     );

        //     std::cout <<" Close wall with start time: " << std::ctime(&start_time_t) << std::endl;
        //     std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
        //     std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;
        //     isCloseToWall = false;
        // }

        if ( dist_obs > -1.0f ){      
            if ( nTimer <= 3000 ){
                float dist = std::sqrt(std::pow(cur_pos.x - cur_x,2) + std::pow(cur_pos.y,2));
                if ( dist <= 0.05f ){
                    if ( isCloseToBall == false ){
                        std::cout << "Too close to the ball!!" << std::endl;
                        auto closeBallStartTime = std::chrono::high_resolution_clock::now();
                        isCloseToBall = true;
                    }
                }            
                else if ( isCloseToBall ){
                    closeBallEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = closeBallEndTime - closeBallStartTime;
                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(closeBallStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(closeBallEndTime)
                    );
        
                    std::cout <<" Close ball with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;
                    isCloseToBall = false;
                }    
            }
            else if ( nTimer > 6000 && nTimer <= 9000 ){
                float dist = std::sqrt(std::pow(cur_pos.x,2) + std::pow(cur_pos.y - cur_x,2));
                if ( dist <= 0.05f ){
                    if ( isCloseToBall == false ){
                        std::cout << "Too close to the ball!!" << std::endl;
                        auto closeWallStartTime = std::chrono::high_resolution_clock::now();
                        isCloseToBall = true;
                    }
                }           
                else if ( isCloseToBall ){
                    closeBallEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = closeBallEndTime - closeBallStartTime;
                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(closeBallStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(closeBallEndTime)
                    );
        
                    std::cout <<" Close ball with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;
                    isCloseToBall = false;
                }      
            }
        }

        float dist = std::sqrt(std::pow(cur_pos.x - targetx,2) + std::pow(cur_pos.y - targety,2));
        float dist_1 = std::sqrt(std::pow(cur_pos.x - targetx_1,2) + std::pow(cur_pos.y - targety_1,2));
        float dist_chpad = std::sqrt(std::pow(cur_pos.x - chpadx,2) + std::pow(cur_pos.y - chpady,2));
        // std::cout <<" Current dist: " << dist << ", dist1: " << dist_1 << std::endl;

        // For maze
        if ( dist <= 0.3f ){
            nTargetFoundTimer += 1;         
            targetx = -5.0f;
            targety = -5.0f;
        
        }
        else if ( dist_1 <= 0.3f ){
            nTargetFoundTimer += 1;         
            targetx_1 = -5.0f;
            targety_1 = -5.0f;        
        }

        // int nCount = 2; // 2 for maze 3 for rooms
        // if ( nTargetFoundTimer >= nCount ){            
        //     targetx = -5.0f;
        //     targety = -5.0f;
        // }
        frame1.x(targetx);
        frame1.y(targety);        
        frame1.z(1.0f);
        frame3.x(targetx_1);
        frame3.y(targety_1);        
        frame3.z(1.0f);

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

        if ( dist_chpad <= 0.10f ){
            isChpadFound = 1;
        }
        else{
            isChpadFound = 0;
        }
        targetFoundState.target_found_count(nTargetFoundTimer);
        targetFoundState.is_chpad_found(isChpadFound);
        cluon::data::TimeStamp sampleTime;
        od4.send(frame1, sampleTime, 1);
        od4.send(frame2, sampleTime, 2);
        od4.send(frame3, sampleTime, 3);
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