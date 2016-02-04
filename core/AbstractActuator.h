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

#ifndef __common_AbstractActuator_h__
#define __common_AbstractActuator_h__
#endif

#ifndef ACTUATORS_DEFAULT_TIMEOUT
#define ACTUATORS_DEFAULT_TIMEOUT 1000
#endif

namespace common {
	namespace actuators {

	enum ReadingTypes 
	{	READ_ENCODER=0,
		READ_COUNTER=1
	};
      

	class AbstractActuator {

	public:
	AbstractActuator() {};
	virtual ~AbstractActuator() {};
/**
@brief set timeout in ms for the completion of the operation (0 wait undefinitively)
@return 0 if success or an error code
*/
	virtual int setTimeout(uint32_t timeo_ms)=0;
/**
@brief get timeout in ms for the completion of the operation 
@return 0 if success or an error code
*/
	virtual int getTimeout(uint32_t* timeo_ms)=0;
/**
@brief Move the actuator X millimeters away from current position
@return 0 if success or an error code
*/
	virtual int moveRelativeMillimeters(double millimeters)=0;
/**
@brief get the actuator position using the readingType mode chosen for reading
@return 0 if success or an error code
*/
	virtual int getPosition(common::actuators::ReadingTypes readingType,double& deltaPosition_mm)=0;
/**
@brief initialize and poweron the motor
@return 0 if success
*/
	virtual int init()=0;
 /**
   @brief de-initialize the actuator and close the communication
   @return 0 if success
  */

	virtual int deinit()=0;
/**
   @brief returns the SW/FW version of the driver/FW
   @param version returning string
   @return 0 if success or an error code
*/
   virtual int getSWVersion(std::string& version)=0;

    /**
    @brief returns the HW version of the actuator 
    @param version returning string
    @return 0 if success or an error code
    */
    virtual int getHWVersion(std::string& version)=0;

    virtual int stopMotion()=0;
    virtual int homing()=0;
    virtual int getState(int* state, std::string& desc )=0;
}
}
}
