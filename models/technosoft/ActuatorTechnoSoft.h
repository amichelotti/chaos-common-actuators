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
                    
                public:
                    typedef struct __technoinfo {
                        //const double range;
                        const double mechanicalReduceFactor;
                        const double movementUnit_mm;
                        const int encoderLines;
                        // SerialCommChannelTechnosoft parameters:
                        const std::string pszDevName;
                        const BYTE btType;
                        const DWORD baudrate;
                    } technoinfo_t;
                    
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
                int moveAbsoluteMillimeters(double mm);
                int poweron(uint32_t timeo_ms=ACTUATORS_DEFAULT_TIMEOUT){return 0;}
                int resetAlarms(uint64_t alrm){return 0;}
                
                //int getPosition(readingTypes mode, double& deltaPosition_mm);
                int stopMotion(){return 0;}
                int getPosition(readingTypes mode, float* deltaPosition_mm);
                int homing(homingType mode){return 0;}
                int getState(int* state, std::string& desc );   // **
                int getAlarms(uint64_t*alrm);
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
        int getSWVersion(std::string& version){return 0;}

        /**
        @brief returns the HW version of the actuator 
        @param version returning string
        @return 0 if success or an error code
        */
        int getHWVersion(std::string& version){return 0;}         // ****Da implementare***

            
            };
	}
    }
}
#endif /* ActuatorTechnoSoft_h */
