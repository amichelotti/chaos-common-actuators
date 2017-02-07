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
                
            // ******** Primi 8 bit dedicati ai warning **********
            ACTUATOR_I2T_WARNING_MOTOR = 0x1,
            ACTUATOR_I2T_WARNING_DRIVE = 0x2,
    
            // ******** dal nono bit in poi ci sono gli stati *******
            ACTUATOR_READY = 0x100, 

            ACTUATOR_OVER_POSITION_TRIGGER = 0x200, // Position trigger
            ACTUATOR_AUTORUN_ENABLED = 0x400, // Auto run mode status
            ACTUATOR_LSP_EVENT_INTERRUPUT = 0x800, // Limit switch positive event/interrupt
            ACTUATOR_LSN_EVENT_INTERRUPT = 0x1000, // Limit switch negative event/interrupt
           
            ACTUATOR_IN_GEAR = 0x2000, // Gear ratio in electronic gearing mode
            ACTUATOR_IN_CAM = 0x4000, // Reference position in absolute electronic camming mode
            ACTUATOR_FAULT=0x8000, // Fault status

            // Low part of the status
            ACTUATOR_INMOTION = 0x10000, 
            ACTUATOR_POWER_SUPPLIED = 0x20000,  // cambiare nome
            HOMING_IN_PROGRESS = 0x40000,
            ACTUATOR_LSP_LIMIT_ACTIVE=0x80000,        
            ACTUATOR_LSN_LIMIT_ACTIVE=0x100000,
                    
            // Unknown status
            ACTUATOR_UNKNOWN_STATUS = 0x200000  // Unknown state of the actuator
        } actuatorStatus;

        typedef enum {
            ACTUATOR_CANBUS_ERROR=0x1, // CAN bus status
            ACTUATOR_SHORT_CIRCUIT=0x2, // Short-circuit protection status
            ACTUATOR_INVALID_SETUP_DATA=0x4, // Setup table status
            ACTUATOR_CONTROL_ERROR=0x8, // Control error
            ACTUATOR_SERIAL_COMM_ERROR=0x10, // Communication error
            ACTUATOR_HALL_SENSOR_MISSING=0x20, // Hall sensor missing / Resolver error / BiSS error / Position wrap around error

            //ACTUATOR_LSP_LIMIT_ACTIVE=0x40, // Positive limit switch status
            //ACTUATOR_LSN_LIMIT_ACTIVE=0x80, // Negative limit switch status
            ACTUATOR_OVER_CURRENT=0x40, // Over-current error
            ACTUATOR_I2T=0x80, // I2T protection error
            ACTUATOR_OVERTEMP_MOTOR=0x100, // Motor over temperature error
            ACTUATOR_OVERTEMP_DRIVE=0x200, // Drive over temperature error
            ACTUATOR_OVERVOLTAGE=0x400, // Over voltage error
            ACTUATOR_UNDERVOLTAGE=0x800, // Under voltage error
            ACTUATOR_COMMANDERROR=0x1000, // Command error
            //ACTUATOR_ENABLE_INPUT_ACTIVE // Enable status of drive/motor
            //ACTUATOR_I2T_WARNING_MOTOR = 0x8000, // Motor I2T protection warning
            //ACTUATOR_I2T_WARNING_DRIVE = 0x10000, // Drive I2T protection warning
            ACTUATOR_ALARMS_READING_ERROR = 0x2000,
            ACTUATOR_ALARMS_EMERGENCY_ERROR = 0X4000
        } actuatorAlarms;

    class AbstractActuator {

        protected:
            double range_mm; //mechanical range of the slit (passato da MDS), [mm]
            uint64_t timeo_ms; // ***** DA ELIMINARE *****
            uint64_t timeo_homing_ms; // ***** DA ELIMINARE *****            

        public:
        AbstractActuator() {timeo_ms=0;}; // ***** DA ELIMINARE il corpo *****
        virtual ~AbstractActuator() {};
        
        virtual int setParameter(int axisID, std::string parName,std::string value)=0; // (2)
        
        virtual int moveRelativeMillimeters(int axisID,double mm)=0;  // (2)

virtual int getParameter(int axisID,std::string parName,std::string& resultString)=0;

        /**
        @brief get the actuator position using the readingType mode chosen for reading
        @return 0 if success or an error code
        */
        typedef enum
        {   READ_ENCODER,
            READ_COUNTER,
            READ_POTENTIOMETER
        } readingTypes;

        virtual int getPosition(int axisID,readingTypes mode,double *deltaPosition_mm)=0;   //***OK***

        /**
        @brief initialize and poweron the motor
        @return 0 if success
        */
        virtual int init(void*)=0;
         /**
           @brief de-initialize the actuator and close the communication
           @return 0 if success
          */
            
        virtual int configAxis(void*initialization_string)=0;

        virtual int deinit(int axisID)=0; // (2)      
        virtual int hardreset(int axisID, bool mode)=0;
            
        /**
           @brief returns the SW/FW version of the driver/FW
           @param version returning string
           @return 0 if success or an error code
        */
        virtual int getSWVersion(int axisID, std::string& version)=0;        

        /**
        @brief returns the HW version of the actuator
        @param version returning string
        @return 0 if success or an error code
        */
        virtual int getHWVersion(int axisID, std::string& version)=0;         // ****Da implementare***

/**
@brief returns a string containing the dataset attributes from driver which responds to SetParameter method

*/
        virtual int sendDataset(std::string& dataset)=0; // ****Da implementare***  (2)
         /**

        @brief stop the motion of the actuator 

        @return 0 if success or an error code
        */
        virtual int stopMotion(int axisID)=0;    // (2)                          

        typedef enum{   // Importante non cambiare l'ordine di tali elementi
            defaultHoming,
            homing2,
            nativeHoming15
        } homingType;

        virtual int homing(int axisID,homingType mode)=0;                        // (2) 
        virtual int getState(int axisID,int* state, std::string& desc)=0;        // (2) 
        virtual int getAlarms(int axisID,uint64_t*alrm,std::string& desc)=0;     // (2) 
        virtual int resetAlarms(int axisID,uint64_t alrm)=0;                     // (2)
        virtual int poweron(int axisID,int on)=0;
        virtual uint64_t getFeatures()=0;
        virtual int moveAbsoluteMillimeters(int axisID,double mm)=0;
    };
}
}
#endif
