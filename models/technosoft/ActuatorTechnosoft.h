//
//  ActuatorTechnosoft.h
//  prova
//
//  Created by MacBookProINFN on 29/01/16.
//  Copyright © 2016 MacBookProINFN. All rights reserved.
//

#include "./core/Slit.h"

#ifndef ActuatorTechnosoft_h
#define ActuatorTechnosoft_h

namespace common{
    
    namespace actuators{

            class Actuator: public common::Slit{
    
                private:
                    common::actuators::technosoft::TechnoSoftLowDriver *driver=NULL;
                    char actuator_name[20]; //(passato da MDS) es. SLTTB001 left
                    double movementUnit_mm; // 1.5 mm or 1 mm (MDS)
                    int encoderLines; // (passato da MDS)
                public:
                    // costruttore
                    Actuator(const std::string& actuator_name){     // OK
                          strcpy(this->actuator_name,actuator_name.c_str());
                    }
                    ~Actuator(){}                                     // OK
                    int init(const double& range,const double& mechanicalReduceFactor,const double& movementUnit_mm,const int& encoderLines,const std::string& filePath,const int& axisID, const double& speed, const double& acceleration, const BOOL& isAdditive, const short& moveMoment, const short& referenceBase);// all'interno di initActuator dovra essere richiamata la funzione initTechnoSoft
                    void deinit();
                    int moveRelativeMillimeters(double deltaMillimeters);
                    int getPosition(BOOL readingType, double& deltaPosition_mm);
            };
    }
}
#endif /* ActuatorTechnosoft_h */