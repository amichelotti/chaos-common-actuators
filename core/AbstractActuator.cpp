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
#define TEST_STATUS(x,str) if(status&x){desc+=str;}

 std::string AbstractActuator::StatusDecoding(uint64_t status)
 {
	 std::string desc = "";
	 TEST_STATUS(ACTUATOR_I2T_WARNING_MOTOR,"I2T Motor Warning|");
	 TEST_STATUS(ACTUATOR_I2T_WARNING_DRIVE,"I2T Drive Warning|");
	 TEST_STATUS(ACTUATOR_READY,"Actuator Ready|");
	 TEST_STATUS(ACTUATOR_OVER_POSITION_TRIGGER,"Over Position Trigger|");
	 TEST_STATUS(ACTUATOR_AUTORUN_ENABLED,"Autorun Enabled|");
	 TEST_STATUS(ACTUATOR_LSP_EVENT_INTERRUPUT,"Positive Limit Switch Event Occurred|");
	 TEST_STATUS(ACTUATOR_LSN_EVENT_INTERRUPT,"Negative Limit Switch Event Occurred|");
	 TEST_STATUS(ACTUATOR_IN_GEAR,"Gearing Mode|");
	 TEST_STATUS(ACTUATOR_IN_CAM,"Camming Mode|");
	 TEST_STATUS(ACTUATOR_FAULT,"Actuator in Fault|");
	 TEST_STATUS(ACTUATOR_INMOTION,"Actuator in Motion|");
	 TEST_STATUS(ACTUATOR_POWER_SUPPLIED,"Actuator Power ON|");
	 TEST_STATUS(HOMING_IN_PROGRESS,"Homing in Progress|");
	 TEST_STATUS(ACTUATOR_LSP_LIMIT_ACTIVE,"Positive Limit Switch Active|");
	 TEST_STATUS(ACTUATOR_LSN_LIMIT_ACTIVE,"Negative Limit Switch Active|");
	 TEST_STATUS(ACTUATOR_AT_HOME,"At Home|");
	 TEST_STATUS(ACTUATOR_HOMED,"Home Done|");
	 TEST_STATUS(ACTUATOR_ENCODER,"Actuator has encoder|");
	 TEST_STATUS(ACTUATOR_MOTION_COMPLETE,"Motion Done|");
	 TEST_STATUS(ACTUATOR_COMM_ERROR,"Communication Error|");
	 TEST_STATUS(ACTUATOR_STALL_ERROR,"Stall Error|");
	 TEST_STATUS(ACTUATOR_HOME_LIMIT,"Home Limit|");
	 TEST_STATUS(ACTUATOR_GAIN_CTRL,"Actuator has position ctrl|");
	 TEST_STATUS(ACTUATOR_POSITION_CTRL,"Actuator has close loop ctrl");


	 return desc;
 }

//int AbstractActuator::setTimeout(uint64_t _timeo_ms){ // ******* DA ELIMINARE *******
//    timeo_ms=_timeo_ms;
//    return 0;
//}        
//int AbstractActuator::getTimeout(uint64_t* _timeo_ms){ // ******* DA ELIMINARE *******
//    *_timeo_ms=timeo_ms;
//    return 0;
//}
//
//int AbstractActuator::setTimeoutHoming(uint64_t timeo_ms){ // ******* DA ELIMINARE  *******
//    timeo_homing_ms = timeo_ms;
//    return 0;
//}
//int AbstractActuator::getTimeoutHoming(uint64_t* timeo_ms){ // ******* DA ELIMINARE  *******
//    * timeo_ms = timeo_homing_ms;
//    return 0;
//}

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

