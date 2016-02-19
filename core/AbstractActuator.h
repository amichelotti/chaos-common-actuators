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


#ifndef ACTUATORS_DEFAULT_TIMEOUT
#define ACTUATORS_DEFAULT_TIMEOUT 1000
#endif



namespace common {
    namespace actuators {
       
    class AbstractActuator {

        protected:
            double range_mm; //mechanical range of the slit (passato da MDS), [mm]
            uint64_t timeo_ms;
            
            // Trapezoidal profile parameters
            double speed;
            double acceleration;
            bool isAdditive;
            int32_t movement;
            int32_t referenceBase;
            
        public:
        AbstractActuator() {timeo_ms=0;};
        virtual ~AbstractActuator() {};
        /**
        @brief set timeout in ms for the completion of the operation (0 wait undefinitively)
        @return 0 if success or an error code
        */
        int setTimeout(uint64_t timeo_ms);                // ****Da implementare***
        /**
        @brief get timeout in ms for the completion of the operation 
        @return 0 if success or an error code
        */
        int getTimeout(uint64_t* timeo_ms);               // ****Da implementare***
        /**
        @brief Move the actuator X millimeters away from current position
        @return 0 if success or an error code
        */
        virtual int moveRelativeMillimeters(double mm)=0;      //***OK**
        virtual int moveAbsoluteMillimeters(double mm)=0;      //***OK**
            
        /**
        @brief set the actuator speed in mm/s
        @return 0 if success or an error code
        */
         virtual int setSpeed(double speed_mm_per_sec);
            
        /**
        @brief set the actuator acceleration in mm/s^2
        @return 0 if success or an error code
        */
            virtual int setAcceleration(double acceleration_mm_per_sec2);
            
        /**
        @brief specify how is computed the position to reach
        */    
            virtual void setAdditive(bool isAdditive);
            
        /**
        @brief define the moment when the motion is started
        @return 0 if success or an error code
        */    
            virtual int setMovement(int32_t movement);  //********** per ora ritorna un valore ***********
            
        /**
        @brief specify how the motion reference is computed is computed: from 
         * actual values of position and speed reference or from actual values
         * of load/motor position and speed
        @return 0 if success or an error code
        */     
            virtual int setReferenceBase(int32_t referenceBase); //********** per ora ritorna un valore ***********
            
        /**
        @brief get the actuator position using the readingType mode chosen for reading
        @return 0 if success or an error code
        */    
        typedef enum
        {   READ_ENCODER,
            READ_COUNTER
        } readingTypes;
        
        virtual int getPosition(readingTypes mode,double& deltaPosition_mm)=0;   //***OK***
           
        /**
        @brief initialize and poweron the motor
        @return 0 if success
        */
            virtual int init(void*)=0;
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
            
            typedef enum{   
                defaultHoming,
                homing2,   
                nativeHoming15    
            } homingType;
        
            virtual int homing(homingType mode)=0;
            virtual int getState(int* state, std::string& desc )=0;   // ****Da implementare***
    };
}
}
#endif