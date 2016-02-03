//
//  ActuatorTechnoSoft.cpp
//  
//
//  Created by MacBookProINFN on 29/01/16.
//
//

#include <stdio.h>
#include "ActuatorTechnosoft.h"

using namespace common::actuators::technosoft;

int Actuator::init(const double& range,const double& _mechanicalReduceFactor,const double& _movementUnit_mm,const int& _encoderLines,const std::string& filePath, const int& axisID, const double& speed, const double& acceleration, const BOOL& isAdditive, const short& moveMoment, const short& referenceBase){
    
    if (driver!=NULL) {
        delete driver;
    }
    driver = new (std::nothrow) TechnoSoftLowDriver(filePath);
    if (driver==NULL) {
        return -1;
    }
    // Inizializzazione parametri TechnoSoftLowDriver
    driver->init(axisID,speed,acceleration,isAdditive,moveMoment,referenceBase);
    
    // Inizializzazione parametri Actuator
    range_mm = range;
    mechanicalReduceFactor=_mechanicalReduceFactor;
    movementUnit_mm = _movementUnit_mm;
    encoderLines = _encoderLines;
    
    return 0;
}

void Actuator::deinit(){
    
    if (driver!=NULL) {
        delete driver;
    }
}

int Actuator::moveRelativeMillimeters(double deltaMillimeters){
    
    // Calcolo argomento funzione moveRelativeSteps
    double deltaMicroSteps = round((N_ROUNDS*STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    long deltaMicroStepsL = deltaMicroSteps;
    if(!driver->moveRelativeSteps(deltaMicroStepsL))
        return -2;
    
    return 0;
}

int Actuator::getPosition(readingTypes readingType, double& deltaPosition_mm){
    
    if(readingType==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(!driver->getCounter(tposition))
            return -1;
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*N_ROUNDS);
    }
    else if(readingType==READ_ENCODER){               // Lettura posizione per mezzo dell'encoder (Apos register)
        long aposition;
        
        if(!driver->getEncoder(aposition))
            return -2;
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(N_ENCODER_LINES*N_ROUNDS);
    }
    
    return 0;
}
