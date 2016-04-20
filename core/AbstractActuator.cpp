/*
 *      AbstractActuator.h
 *      !CHAOS
 *      Created by Alessandro D'Uffizi
 *
 *      Copyright 2012 INFN, National Institute of Nuclear Physics
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include <inttypes.h>
#include "AbstractActuator.h"

using namespace common::actuators;

int AbstractActuator::setTimeout(uint64_t _timeo_ms){
    timeo_ms=_timeo_ms;
    return 0;
}        
int AbstractActuator::getTimeout(uint64_t* _timeo_ms){
    *_timeo_ms=timeo_ms;
    return 0;
}

//int AbstractActuator::setSpeed(double speed_mm_per_sec){
//     speed=speed_mm_per_sec;
//     return 0;
//}
// 
//int AbstractActuator::setAcceleration(double acceleration_mm_per_sec2){
//    acceleration=acceleration_mm_per_sec2;
//    return 0;
//}
//
//void AbstractActuator::setAdditive(bool _isAdditive){
//    isAdditive=_isAdditive;
//    
//}
//
//int AbstractActuator::setMovement(int32_t _movement){
//    movement=_movement;
//    return 0;
//}
//      
//int AbstractActuator::setReferenceBase(int32_t _referenceBase){
//    referenceBase=_referenceBase;
//    return 0;
//}

