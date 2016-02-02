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

int Actuator::init(const double& range,const double& mechanicalReduceFactor,const double& movementUnit_mm,const int& encoderLines,const std::string& filePath, const int& axisID, const double& speed, const double& acceleration, const BOOL& isAdditive, const short& moveMoment, const short& referenceBase){
    
    if (this->driver!=NULL) {
        delete this->driver;
    }
    this->driver = new (std::nothrow) TechnoSoftLowDriver(filePath);
    if (this->driver==NULL) {
        return -1;
    }
    // Inizializzazione parametri TechnoSoftLowDriver
    this->driver->init(axisID,speed,acceleration,isAdditive,moveMoment,referenceBase);
    
    // Inizializzazione parametri Actuator
    this->range = range;
    this->mechanicalReduceFactor=mechanicalReduceFactor;
    this->movementUnit_mm = movementUnit_mm;
    this->encoderLines = encoderLines;
    
    return 0;
}

void Actuator::deinit(){
    
    if (this->driver!=NULL) {
        delete this->driver;
    }
}

int Actuator::moveRelativeMillimeters(double deltaMillimeters){
    
    // Calcolo argomento funzione moveRelativeSteps
    double deltaMicroSteps = round((N_ROUNDS*STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    long deltaMicroStepsL = deltaMicroSteps;
    if(!moveRelativeSteps(deltaMicroStepsL))
        return -2;
    
    return 0;
}

int Actuator::getPosition(readingTypes readingType, double& deltaPosition_mm){
    
    if(readingType==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(!TechnoSoftLowDriver::getCounter(tposition))
            return -1;
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*N_ROUNDS);
    }
    else if(readingType==READ_ENCODER){               // Lettura posizione per mezzo dell'encoder (Apos register)
        long aposition;
        
        if(!TechnoSoftLowDriver::getEncoder(aposition))
            return -2;
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(N_ENCODER_LINES*N_ROUNDS);
    }
    
    return 0;
}