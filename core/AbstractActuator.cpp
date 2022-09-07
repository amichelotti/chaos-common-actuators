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

 std::string AbstractActuator::StatusDecoding(uint64_t status)
 {
	 std::string desc = "";
	 for (int i = 0; i < 64; i++)
	 {

		 bool check = ((((int64_t) 1) << i) & status) != 0;

		 {
			 switch (i)
			 {
			 case 0: if (check)
				 desc += "I2T Motor Warning "; break;
			 case 1: if (check)
				 desc += "I2T Drive Warning "; break;
			 case 8: if (check)
				 desc += "Actuator Ready "; break;
			 case 9: if (check)
				 desc += "Over Position Trigger "; break;
			 case 10: if (check)
				 desc += "Autorun Enabled "; break;
			 case 11: if (check)
				 desc += "Positive Limit Switch Event Occurred "; break;
			 case 12: if (check)
				 desc += "Negative Limit Switch Event Occurred "; break;
			 case 13: if (check)
				 desc += "Gearing Mode "; break;
			 case 14: if (check)
				 desc += "Camming Mode "; break;
			 case 15: if (check)
				 desc += "Actuator in Fault "; break;
			 case 16: if (check)
				 desc += "Actuator in Motion "; break;
			 case 17: if (check)
				 desc += "Actuator Power ON ";
					else
				 desc += "Actuator Power OFF ";
				 break;
			 case 18: if (check)
				 desc += "Homing in Progress "; break;
			 case 19: if (check)
				 desc += "Positive Limit Switch Active "; break;
			 case 20: if (check)
				 desc += "Negative Limit Switch Active "; break;
			 case 21: if (check)
				 desc += "Actuator Status Unknown "; break;
			 default: break;

			 }
		 }
	 }
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

