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
#if 0
BOOL ActuatorTechnoSoft::homing(int minutes,std::string& mode){
    
    if(mode.compare("mode_1")==0){
    /*	Settings of homing parameters for this example */
	double high_speed = 10;		/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling] */
	double low_speed = 1.;		/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
	double acceleration = 0.3;	/* the acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
	double deceleration = 1.5;	/* the decceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
	long home_position = 1000;	/* the homing position [drive internal position units, encoder counts]  */
	time_t long_time;
	struct tm *ini_time;
	struct tm *new_time;
	int iMinute, nMinute;
	WORD homing_done = 0;
        
/*	Set the homing parameters */
/*	--------------------------------------------------------------------*/
/*	Set the acceleration rate for homing */
	if(!driver->setFixedVariable("CACC", acceleration)) 
		return -1;		
/*	Set the deceleration rate for homing */
	if(!driver->setDecelerationParam(deceleration)) 
		return -2;	
/*	Set the high speed for homing */
	if(!driver->setFixedVariable("CSPD", high_speed)) 
		return -3;		
/*	Set the low speed for homing */
	if(!driver->setFixedVariable("HOMESPD", low_speed)) 
		return -4;	
/*	Setup the home position at the end of the homing procedure */
	if(!driver->setVariable("HOMEPOS", home_position)) 
		return -5;	
/*	--------------------------------------------------------------------*/

/*	Call the homing procedure stored in the non-volatile memory of the drive */
	/*if(!driver->executeTMLfunction("Homing15"))
		return -6;
*/
/*	Wait until the homing process is ended */
	time( &long_time );
	ini_time = localtime( &long_time );
	iMinute = ini_time->tm_min;
	
/*	Wait for homing procedure to end */	
	while(homing_done == 0){
/*
 * Check the SRL.8 bit - Homing active flag*/

		if(!TS_ReadStatus(REG_SRL, &homing_done)){
                    /*	Cancel the homing procedure   */
                    if(!driver->abortNativeOperation()) 
                        return -7;
                    /*	Stop the motor */
                    if(!driver->stopMotion()) 
			return -8;
                    return -9;
                }

		homing_done=((homing_done & 1<<8) == 0 ? 1 : 0);

/*	If the homing procedure doesn't ends in MINUTES time then abort it */
		time( &long_time );
		new_time = localtime( &long_time );
		nMinute = new_time->tm_min;
		if(nMinute - iMinute >= minutes){
			break;
		}
                usleep(1000); // Check each millisecond
	}
        /*if (homing_done != 0){
		printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
		printf(" Homing procedure done!\n");
	}
         * */
        if(homing_done == 0){
        /*	Cancel the homing procedure	*/
		if(!driver->abortNativeOperation()) 
			return -10;

        /*	Stop the motor */
		if(!driver->stopMotion()) 
			return -11;

		//printf(" After %d [min] the homing procedure was not finished!\n", minutes);
                
                return -12;
	}	
	return 0;
    }
    else if(mode.compare("mode_2")==0){
        /*	Movement parameters	*/
	long position = -100000000;		/* position command [drive internal position units, encoder counts] */
	long home_position = 1000;		/* the homing position [drive internal position units, encoder counts] */
	long cap_position = 0;			/* the position captures at HIGH-LOW transition of negative limit switch */
	//double high_speed = 10;			/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling]*/
	double low_speed = 1.0;			/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
	//double acceleration = 0.6;		/* acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
        
/*	Command a trapezoidal positioning to search the negative limit switch */
   
        //if(!TS_MoveRelative(position, speed, acceleration, isAdditive,movement,referenceBase)) 
		//return FALSE;
        
        // ATTENZIONE: 
        // Adesso position sarà inteso in mm
        // ******** E' necessario settare i membri isAdditive,movement,referenceBase 
        // al valore desiderato.************
        
        if(!moveRelativeMillimeters(position))
            return -15;
	
/*	Wait for the HIGH-LOW transition on negative limit switch */
	if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, WAIT_EVENT, NO_STOP)) 
		return FALSE;

/*	Wait until the motor stops */
	if(!TS_SetEventOnMotionComplete(WAIT_EVENT,NO_STOP)) 
		return FALSE;

/*	Read the captured position on limit switch transition */
	if(!TS_GetLongVariable("CAPPOS", &cap_position)) return FALSE;
	printf(" The captured position is: %ld [drive internal position units]\n", cap_position);

/*	Command an absolute positioning on the captured position */
	if(!TS_MoveAbsolute(cap_position, low_speed, acceleration, UPDATE_IMMEDIATE, FROM_REFERENCE)) 
		return FALSE;

/*	Wait for positioning to end */
	if(!TS_SetEventOnMotionComplete(WAIT_EVENT,NO_STOP)) 
		return FALSE;

/*	Set the home position */
	if(!TS_SetPosition(home_position)) 
		return FALSE;
	
	printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
	printf(" Homing procedure done!\n");

	return TRUE;
    }
}
#endif
