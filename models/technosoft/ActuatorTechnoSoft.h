//
//  ActuatorTechnoSoft.h
//  prova
//
//  Created by MacBookProINFN on 29/01/16.
//  Copyright © 2016 MacBookProINFN. All rights reserved.
//

#include <common/actuators/core/AbstractActuator.h>
#include "TechnoSoftLowDriver.h"
#include <sys/time.h>

//#define CONST_MULT_TECHNOFT_DEFAULT 256.0 // numero micro steps per step (MDS)
//#define STEPS_PER_ROUNDS_DEFAULT 200.0     // numero steps per giro
//#define N_ROUNDS_DEFAULT 20.0 
//#define LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT 1.5 //[mm]


#ifndef ActuatorTechnoSoft_h
#define ActuatorTechnoSoft_h

namespace common{
    
    namespace actuators{

	namespace models{

          class ActuatorTechnoSoft: public ::common::actuators::AbstractActuator{
    
                private:
                    TechnoSoftLowDriver *driver;
                    std::string dev; // Serial channel name
                    std::string name; // ActuatorTechnoSoft name
                    double movementUnit_mm; // 1.5 mm or 1 mm (MDS) 
		    double mechanicalReduceFactor; // fattore di riduzione albero motore/slitta
                    bool readyState;
                    
                    //double highSpeedHoming; // The homing travel speed
                    //double range;
                     
                public:
//                    typedef struct __technoinfo {
//                        //const double range;
//                        const double mechanicalReduceFactor;
//                        const double movementUnit_mm;
//                        const int encoderLines;
//                        // SerialCommChannelTechnosoft parameters:
//                        const std::string pszDevName;
//                        const BYTE btType;
//                        const DWORD baudrate;
//                    } technoinfo_t;
                    
                // costruttore
                ActuatorTechnoSoft();
                ~ActuatorTechnoSoft(){
                    deinit();
                } 
               
        int init(void*initialization_string);
        // OK
        // all'interno di initActuator dovra essere richiamata la funzione initTechnoSoft
        int deinit();
                int moveRelativeMillimeters(double deltaMillimeters);
                int moveRelativeMillimetersHoming(double deltaMillimeters);
                int moveVelocityHoming();
                
                int setTrapezoidalProfile(double speed, double acceleration, bool isAdditive, int32_t movement, int32_t referenceBase);
                int setSpeed(double speed);
                int setAcceleration(double acceleration);
                int setAdditive(bool isAdditive); // NOTA: bool dovra' essere castato a int
                int setMovement(int32_t movement); // NOTA: int32_t dovra' essere castato a short
                int setReferenceBase(int32_t referenceBase); // NOTA: int32_t dovra' essere castato a short
                
                int moveAbsoluteMillimeters(double mm);
                int moveAbsoluteMillimetersHoming(double mm);
                
                int poweron(uint32_t timeo_ms=ACTUATORS_DEFAULT_TIMEOUT){return 0;}
                int resetAlarms(uint64_t alrm){return 0;}
                
                //int getPosition(readingTypes mode, double& deltaPosition_mm);
                int stopMotion();
                int getPosition(readingTypes mode, double* deltaPosition_mm);
                int homing(homingType mode);
                int getState(int* state, std::string& desc );   // **
                int getAlarms(uint64_t*alrm, std::string& descStr);
                uint64_t getFeatures(){return 0;}
     
        /**
        @brief get the actuator position using the readingType mode chosen for reading
        @return 0 if success or an error code
        */    
           
        /**
           @brief returns the SW/FW version of the driver/FW
           @param version returning string
           @return 0 if success or an error code
        */
	int getSWVersion(std::string& version){version="technosoft-0.0.1";return 0;}

        /**
        @brief returns the HW version of the actuator 
        @param version returning string
        @return 0 if success or an error code
        */
        int getHWVersion(std::string& version){version="technosofthw-0.0";return 0;}

            
            };
	}
    }
}
#endif /* ActuatorTechnoSoft_h */
