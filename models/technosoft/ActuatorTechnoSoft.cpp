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

int ActuatorTechnoSoft::moveAbsoluteMillimeters(double millimeters){ 
    
    // Calcolo argomento funzione moveAbsoluteSteps
    double nMicroSteps = round((N_ROUNDS*STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS);
    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    long nMicroStepsL = nMicroSteps;
    if(!driver->moveAbsoluteSteps(nMicroStepsL))
        return -2;
    
    return 0;
}

int ActuatorTechnoSoft::getPosition(readingTypes mode, double& deltaPosition_mm){
    if(mode==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(!driver->getCounter(tposition)){
            DERR("getting counter");
            return -1;
        }
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*N_ROUNDS);
    }
    else if(mode==READ_ENCODER){               // Lettura posizione per mezzo dell'encoder (Apos register)
        long aposition;
        
        if(!driver->getEncoder(aposition))
            return -2;
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(N_ENCODER_LINES*N_ROUNDS);
    }
    return 0;
}

int ActuatorTechnoSoft::homing(homingType mode){
    
    struct timeval structTimeEval;
    if(mode==nativeHoming15){
    /*	Settings of homing parameters for this example */
	double high_speed = 10;		/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling] */
	double low_speed = 1.;		/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
	double acceleration = 0.3;	/* the acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
	double deceleration = 1.5;	/* the decceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
	long home_position = 1000;	/* the homing position [drive internal position units, encoder counts]  */
	WORD homing_done = 0;
        long currentTime_ms;
        
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
	if(!driver->executeTMLfunction("Homing15")) 
		return -6;

/*	Wait until the homing process is ended */
        gettimeofday(&structTimeEval, NULL);
        long iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
	
/*	Wait for homing procedure to end */	
	while(homing_done == 0){
/*	Check the SRL.8 bit - Homing active flag*/
		if(!TS_ReadStatus(REG_SRL, &homing_done)){ 
                    /*	Cancel the homing procedure   */
                    if(!driver->abortNativeOperation()) 
                        return -7; // HARD ERROR
                    /*	Stop the motor */
                    if(!driver->stopMotion()) 
			return -8; // HARD ERROR
                    return -9;
                }
		homing_done=((homing_done & 1<<8) == 0 ? 1 : 0);

/*	If the homing procedure doesn't ends in MINUTES time then abort it */
                gettimeofday(&structTimeEval, NULL);
                currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 100;
		if(currentTime_ms - iniTime_ms >= timeo_ms){
			break;
		}
                usleep(1000); // Check each millisecond
	}//chiude while(homing_done == 0)
        
        if(homing_done == 0){
        /*	Cancel the homing procedure	*/
		if(!driver->abortNativeOperation()) 
			return -10;

        /*	Stop the motor */
		if(!driver->stopMotion()) 
			return -11;
                
                return -12;
	}	
	return 0;
    }
    else if(mode==homing2){
        /*	Movement parameters	*/
	long position_mm = -100000000;		/* position command [drive internal position units, encoder counts] */
	long home_position = 1000;		/* the homing position [drive internal position units, encoder counts] */
	double high_speed = 10;			/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling]*/
	double low_speed = 1.0;			/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
	double acceleration = 0.6;		/* acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
        
        double iniTime_ms;
        double currentTime_ms;
        BOOL wait_event = FALSE; 
        BOOL no_stop = FALSE; 
        BOOL switchTransited = FALSE;
        
         
        double tol_ms = 0.000000001;
        double perc1=0.8;
        double perc2=0.1;
        double perc3=0.1;
        // nota: perc1+perc2+perc3=1
        
        long cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
        
        /* Command a trapezoidal positioning to search the negative limit switch */
   
        //if(!TS_MoveRelative(position,high_speed, acceleration, NO_ADDITIVE,UPDATE_IMMEDIATE,FROM_REFERENCE)) 
		//return FALSE;
        
        // ATTENZIONE: 
        // Adesso position sarà inteso in mm
        // ******** E' necessario settare i membri isAdditive,movement,referenceBase 
        // al valore desiderato.************
        
        // position -> position_mm    
        // high_speed -> membro speed OK
        // acceleration -> member acceleration OK
        // member isAdditive=0; 
        // member movement = UPDATE_IMMEDIATE;
        // member referenceBase = FROM_REFERENCE;
        
        if(moveRelativeMillimeters(position_mm)!=0)
            return -1;
	
        /*	Wait for the HIGH-LOW transition on negative limit switch */
	if(!driver->setEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, wait_event, no_stop)) 
            return -2;
        
        gettimeofday(&structTimeEval, NULL);
        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        currentTime_ms = iniTime_ms+tol_ms;
       
        while(!switchTransited && ((currentTime_ms - iniTime_ms) < timeo_ms*perc1)){
            
            if(!driver->checkEvent(switchTransited))
                return -3;
            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        }
        
        BOOL motionCompleted = FALSE;
        if(switchTransited){
            /*	Wait until the motor stops */
            if(!driver->setEventOnMotionComplete(wait_event,no_stop)) 
                return -4;
            gettimeofday(&structTimeEval, NULL);
            iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
            currentTime_ms = iniTime_ms+tol_ms;
            while(!motionCompleted && (currentTime_ms - iniTime_ms < timeo_ms*perc2)){
                if(!driver->checkEvent(motionCompleted))
                    return -5;
            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
            }
        }
        else 
            return -50; // valore che scegliamo per denotare la scadenza del timeout
        
        if(motionCompleted){
            /*	Read the captured position on limit switch transition */
            if(!driver->getLVariable("CAPPOS", &cap_position)) 
                return -6;
            //printf(" The captured position is: %ld [drive internal position units]\n", cap_position);
        }
        else
            return -50; // valore che scegliamo per denotare la scadenza del timeout

        /*	Command an absolute positioning on the captured position */
	if(moveAbsoluteMillimeters(cap_position)!=0); 
            return -7;

        /*	Wait for positioning to end */
        
	if(!driver->setEventOnMotionComplete(wait_event,no_stop)) 
            return -8;
        gettimeofday(&structTimeEval, NULL);
        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        currentTime_ms = iniTime_ms+tol_ms;
        
        BOOL absoluteMotionCompleted = FALSE;
        while(!absoluteMotionCompleted && (currentTime_ms - iniTime_ms < timeo_ms*perc3)){
             if(!driver->checkEvent(absoluteMotionCompleted))
                    return -9;
            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        }
        
        /*	Set the home position */
        if(absoluteMotionCompleted){
            if(!driver->setPosition(home_position)) 
            return -10;
        }
        else 
            return -50; // valore che scegliamo per denotare la scadenza del timeout
        //printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
	//printf(" Homing procedure done!\n");
	return 0;
    }
    else if(mode==defaultHoming){
        
        
    }
    return -50; // Indica che non ho indicato alcuna modalità corretta
}

int getSWVersion(std::string& version){
    
    /*	Read the firmware version programmed on the drive */
    if (!MSK_GetDriveVersion(FW_version)) 
	return FALSE;
    printf("\n The drive is programmed with %s firmware.\n\n", FW_version);
        
    return 0;
}
