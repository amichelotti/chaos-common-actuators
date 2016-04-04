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

        
    typedef enum {   
        // High part of the status
        //ACTUATOR_END_INIT = 0x1, // Bit per la gestione della lettura degli altri bit di stato 
        
        //AGGIUNGERE STATO DI MOTORE PRONTO ALL'UTILIZZO:
        ACTUATOR_READY = 0x1,
        
        ACTUATOR_OVER_POSITION_TRIGGER = 0x2, // Position trigger
        ACTUATOR_AUTORUN_ENABLED = 0x4, // Auto run mode status
        ACTUATOR_LSP_EVENT_INTERRUPUT = 0x8, // Limit switch positive event/interrupt
        ACTUATOR_LSN_EVENT_INTERRUPT = 0x10, // Limit switch negative event/interrupt
        //ACTUATOR_CAPTURE_EVENT_INTERRUPT = 0x20, // Capture event/interrupt
        //ACTUATOR_TARGET_REACHED = 0x40, // Target command
        //ACTUATOR_I2T_WARNING_MOTOR = 0x80, // Motor I2T protection warning
        //ACTUATOR_I2T_WARNING_DRIVE = 0x100, // Drive I2T protection warning
        ACTUATOR_IN_GEAR = 0x20, // Gear ratio in electronic gearing mode
        ACTUATOR_IN_CAM = 0x40, // Reference position in absolute electronic camming mode
        ACTUATOR_FAULT=0x80, // Fault status
        
        // Low part of the status
        ACTUATOR_MOTION_COMPLETED = 0x100,
        ACTUATOR_POWER_SUPPLIED = 0x200,  // cambiare nome
                
        // Unknown status
        ACTUATOR_UNKNOWN_STATUS // Unknown state of the actuator
    } actuatorStatus;
    

    typedef enum {
        ACTUATOR_CANBUS_ERROR=0x1, // CAN bus status
        ACTUATOR_SHORT_CIRCUIT=0x2, // Short-circuit protection status
        ACTUATOR_INVALID_SETUP_DATA=0x4, // Setup table status
        ACTUATOR_CONTROL_ERROR=0x8, // Control error
        ACTUATOR_SERIAL_COMM_ERROR=0x10, // Communication error
        ACTUATOR_HALL_SENSOR_MISSING=0x20, // Hall sensor missing / Resolver error / BiSS error / Position wrap around error 

        ACTUATOR_LSP_LIMIT_ACTIVE=0x40, // Positive limit switch status
        ACTUATOR_LSN_LIMIT_ACTIVE=0x80, // Negative limit switch status
        ACTUATOR_OVER_CURRENT=0x100, // Over-current error
        ACTUATOR_I2T=0x200, // I2T protection error
        ACTUATOR_OVERTEMP_MOTOR=0x400, // Motor over temperature error
        ACTUATOR_OVERTEMP_DRIVE=0x800, // Drive over temperature error
        ACTUATOR_OVERVOLTAGE=0x1000, // Over voltage error
        ACTUATOR_UNDERVOLTAGE=0x2000, // Under voltage error
        ACTUATOR_COMMANDERROR=0x4000, // Command error
        //ACTUATOR_ENABLE_INPUT_ACTIVE // Enable status of drive/motor
        ACTUATOR_I2T_WARNING_MOTOR = 0x8000, // Motor I2T protection warning
        ACTUATOR_I2T_WARNING_DRIVE = 0x10000, // Drive I2T protection warning
    } actuatorAlarms;

    class AbstractActuator {

        protected:
            double range_mm; //mechanical range of the slit (passato da MDS), [mm]
            uint64_t timeo_ms;
            uint64_t timeo_homing_ms;

            // Trapezoidal profile parameters
            //double speed;
            //double acceleration;
            //bool isAdditive;
            //int32_t movement;
            //int32_t referenceBase;

        public:
        AbstractActuator() {timeo_ms=0;};
        virtual ~AbstractActuator() {};
        /**
        @brief set timeout in ms for the completion of the operation (0 wait undefinitively)
        @return 0 if success or an error code
        */
        int setTimeout(uint64_t timeo_ms);
        /**
        @brief get timeout in ms for the completion of the operation
        @return 0 if success or an error code
        */
        int getTimeout(uint64_t* timeo_ms);
        
        // Set homing procedure time out
        int setHomingTimeout(uint64_t timeo_ms);
        
        // Get homing procedure time out
        int getHomingTimeout(uint64_t* timeo_ms);
        
        /**
        @brief Move the actuator X millimeters away from current position
        @return 0 if success or an error code
        */
        virtual int moveRelativeMillimeters(double mm)=0;

//        /**
//        @brief set the actuator speed in mm/s
//        @return 0 if success or an error code
//        */
//         virtual int setSpeed(double speed_mm_per_sec);

//        /**
//        @brief set the actuator acceleration in mm/s^2
//        @return 0 if success or an error code
//        */
//            virtual int setAcceleration(double acceleration_mm_per_sec2);

//        /**
//        @brief specify how is computed the position to reach
//        */
//            virtual void setAdditive(bool isAdditive);

//        /**
//        @brief define the moment when the motion is started
//        @return 0 if success or an error code
//        */
//            virtual int setMovement(int32_t movement);  //********** per ora ritorna un valore ***********

//        /**
//        @brief specify how the motion reference is computed is computed: from
//         * actual values of position and speed reference or from actual values
//         * of load/motor position and speed
//        @return 0 if success or an error code
//        */
//            virtual int setReferenceBase(int32_t referenceBase); //********** per ora ritorna un valore ***********

        /**
        @brief get the actuator position using the readingType mode chosen for reading
        @return 0 if success or an error code
        */
        typedef enum
        {   READ_ENCODER,
            READ_COUNTER
        } readingTypes;

        virtual int getPosition(readingTypes mode,float *deltaPosition_mm)=0;   //***OK***

        /**
        @brief initialize and poweron the motor
        @return 0 if success
        */
            virtual int init(void*)=0;
         /**
           @brief de-initialize the actuator and close the communication
           @return 0 if success
          */

            virtual int deinit()=0;                                    // ****Da implementare***
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

         /**

        @brief stop the motion of the actuator 

        @return 0 if success or an error code
        */
            virtual int stopMotion()=0;

            typedef enum{
                defaultHoming,
                homing2,
                nativeHoming15
            } homingType;

            virtual int homing(homingType mode)=0;
            virtual int getState(int* state, std::string& desc)=0;   // ****Da implementare***
            virtual int getAlarms(uint64_t*alrm)=0;
            virtual int resetAlarms(uint64_t alrm)=0;
            virtual int poweron(uint32_t timeo_ms=ACTUATORS_DEFAULT_TIMEOUT)=0;
            virtual uint64_t getFeatures()=0;
            virtual int moveAbsoluteMillimeters(double mm)=0;
    };
}
}
#endif
