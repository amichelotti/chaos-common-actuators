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
#include <string>

#ifndef __common_AbstractActuator_h__
#define __common_AbstractActuator_h__
#endif

#ifndef ACTUATORS_DEFAULT_TIMEOUT
#define ACTUATORS_DEFAULT_TIMEOUT 1000
#endif



namespace common {
    namespace actuators {
        typedef enum
        {	READ_ENCODER,
            READ_COUNTER
        } readingTypes;
    
    class Slit {

        protected:
            double range_mm; //mechanical range of the slit (passato da MDS), [mm]
        public:
        Slit() {};
        virtual ~Slit() {};
        /**
        @brief set timeout in ms for the completion of the operation (0 wait undefinitively)
        @return 0 if success or an error code
        */
            virtual int setTimeout(uint64_t timeo_ms)=0;                // ****Da implementare***
        /**
        @brief get timeout in ms for the completion of the operation 
        @return 0 if success or an error code
        */
            virtual int getTimeout(uint64_t* timeo_ms)=0;               // ****Da implementare***
        /**
        @brief Move the actuator X millimeters away from current position
        @return 0 if success or an error code
        */
            virtual int moveRelativeMillimeters(double millimeters)=0;      //***OK***
        /**
        @brief get the actuator position using the readingType mode chosen for reading
        @return 0 if success or an error code
        */
            virtual int getPosition(readingTypes readingType,double& deltaPosition_mm)=0;                                        //***OK***
        /**
        @brief initialize and poweron the motor
        @return 0 if success
        */
            virtual int init()=0;
         /**
           @brief de-initialize the actuator and close the communication
           @return 0 if success
          */

            virtual void deinit()=0;                                    // ****Da implementare***
        /**
           @brief returns the SW/FW version of the driver/FW
           @param version returning string
           @return 0 if success or an error code
        */
       virtual int getSWVersion(std::string& version)=0;          // ****Da implementare***

        /**
        @brief returns the HW version of the actuator 
        @param version returning string
        @return 0 if success or an error code
        */
        virtual int getHWVersion(std::string& version)=0;         // ****Da implementare***

        virtual int stopMotion()=0;
        virtual int homing()=0; // ***Da implementare***
        virtual int getState(int* state, std::string& desc )=0;   // ****Da implementare***
    };
}
}
