//
//  ActuatorTechnoSoft.cpp
//  
//
//  Created by MacBookProINFN on 29/01/16.
//
//

#include <stdio.h>
#include "ActuatorTechnoSoft.h"
#include <boost/regex.hpp>

#include <common/debug/core/debug.h>
using namespace common::actuators::models;
//([\\w\\/]+)int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
      

// initialisation format <device>,<device name>,<configuration path>,<axisid>,
static const boost::regex driver_match("([\\w\\/]+),(\\w+),([\\w\\/\\.]+),(\\d+)");

ActuatorTechnoSoft::ActuatorTechnoSoft(){
    driver=NULL;
}

int ActuatorTechnoSoft::init(void*initialization_string){
    std::string params;
    params.assign((const char*)initialization_string);
    boost::smatch match;

    if(regex_match(params, match, driver_match, boost::match_extra)){
        std::string dev=match[1];      
        std::string dev_name=match[2]; 
        std::string conf_path=match[3];
        std::string axid=match[4];
        driver = new (std::nothrow) TechnoSoftLowDriver(dev,dev_name);
        if((driver)==NULL){
            DERR("## cannot create driver");
            return -1;
        }
        DPRINT("initializing \"%s\" dev:\"%s\" conf path:\"%s\"",dev_name.c_str(),dev.c_str(),conf_path.c_str());
        return driver->init(conf_path,atoi(axid.c_str()));
    }

   DERR("error parsing initialisation string:\"%s\"",params.c_str());
   return -3;
}


void ActuatorTechnoSoft::deinit(){
    DPRINT("deinitializing");
    if (driver!=NULL) {
        delete driver;
    }
}

int ActuatorTechnoSoft::moveRelativeMillimeters(double deltaMillimeters){
        DPRINT("moving relative %f mm",deltaMillimeters);

    // Calcolo argomento funzione moveRelativeSteps
    double deltaMicroSteps = round((N_ROUNDS*STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    long deltaMicroStepsL = deltaMicroSteps;
    if(!driver->moveRelativeSteps(deltaMicroStepsL))
        return -2;
    
    return 0;
}

int ActuatorTechnoSoft::getPosition(readingTypes readingType, double& deltaPosition_mm){
    if(readingType==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(!driver->getCounter(tposition)){
            DERR("getting counter");
            return -1;
        }
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
