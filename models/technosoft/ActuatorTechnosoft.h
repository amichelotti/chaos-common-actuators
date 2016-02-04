//
//  ActuatorTechnosoft.h
//  prova
//
//  Created by MacBookProINFN on 29/01/16.
//  Copyright © 2016 MacBookProINFN. All rights reserved.
//

#include <common/actuators/core/AbstractSlit.h>
#include "TechnoSoftLowDriver.h"

#ifndef ActuatorTechnosoft_h
#define ActuatorTechnosoft_h

namespace common{
    
    namespace actuators{

	namespace models{

          
            
            class ActuatorTechnosoft: public common::actuators::AbstractSlit{
    
                private:
                    common::actuators::technosoft::TechnoSoftLowDriver *driver=NULL;
                    std::string dev;
                    std::string name;
                    double movementUnit_mm; // 1.5 mm or 1 mm (MDS)
		    double mechanicalReduceFactor;
                public:
                      typedef struct __technoinfo {
                        const double range;
                        const double mechanicalReduceFactor;
                        const double movementUnit_mm;
                        const int boh;
                        const std::string pszDevName;
                        const BYTE btType;
                        const DWORD baudrate;
                
                    } technoinfo_t;
                    
                    // costruttore
                    Actuator();
                    ~Actuator(){} 
                    int init(void*initialization_string);
                    // OK
                    int init(const double& range,const double& mechanicalReduceFactor,const double& movementUnit_mm,const int& 				encoderLines,const std::string& filePath, const int& axisID, const double& speed, const double& 			acceleration, const BOOL& isAdditive, const short& moveMoment, const short& referenceBase);
			// all'interno di initActuator dovra essere richiamata la funzione initTechnoSoft
                    void deinit();
                    int moveRelativeMillimeters(double deltaMillimeters);
                    int getPosition(readingTypes readingType, double& deltaPosition_mm);
                    int stopMotion(){return 0;};
                    int homing(){return 0;}; // ***Da implementare***
                    int getState(int* state, std::string& desc ){return 0;}   // **
            };
	}
    }
}
#endif /* ActuatorTechnosoft_h */
