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
using namespace ::common::actuators::models;

//([\\w\\/]+)int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
      

// initialisation format <device>,<device name>,<configuration path>,<axisid>,
static const boost::regex driver_match("([\\w\\/]+),(\\w+),(.+),(\\d+)");

ActuatorTechnoSoft::ActuatorTechnoSoft(){
    driver=NULL;
    readyState=false;
}

int ActuatorTechnoSoft::init(void*initialization_string){
    std::string params;
    if(readyState==true){
      DPRINT("already initialized");
      return 0;
    }
    params.assign((const char*)initialization_string);
    boost::smatch match;

    if(regex_match(params, match, driver_match, boost::match_extra)){
        std::string dev=match[1];      
        std::string dev_name=match[2]; 
        std::string conf_path=match[3];
        std::string axid=match[4];
        try{
            driver = new (std::nothrow) TechnoSoftLowDriver(dev,dev_name);
            // ATTENZIONE: IL COSTRUTTORE TechnoSoftLowDriver DOVRÀ IN SEGUITO 
            // GENERARE UNA ECCEZIONE nel caso l'oggetto SerialCommChannelTechnosoft 
            // non venga allocato.
            // L'istruzione precedente dovrà essere invocata facendo uso di un blocco 
            // try/catch. Nel caso l'eccezione venga catturata dovrà essere restituito
            // un numero negativo
        }
        catch(std::bad_alloc& e){ // Eccezione derivante dal fatto che l'oggetto canale di comunicazione
                                    // non è stato possibile allocarlo.
                                  
            ERR("## cannot create channel");
            delete driver; 
            //readyState = false; //non è necessari questa assegnazione
            return -1;
        }
        if((driver)==NULL){
            ERR("## cannot create driver");
            // readyState = false; // non è necessaria questa assegnazione
            return -2;                     
        }
        
        DPRINT("initializing \"%s\" dev:\"%s\" conf path:\"%s\"",dev_name.c_str(),dev.c_str(),conf_path.c_str());
        //int rt= driver->init(conf_path,atoi(axid.c_str()));
        
        // INIZIALIZZAZIONE CANALE + DRIVE/MOTOR
        if(driver->init(conf_path,atoi(axid.c_str()))<0){
            // Deallocazione oggetto canale ()
            try{
                delete driver; // Nota importante: l'indirizzo del canale 
                           // non è stato memorizzato nella mappa. Essendo l'oggetto
                           // canale puntato da un solo oggetto TechnoSOftLowDriver,
                           // anche l'oggetto canale sarà deallocato (funzionamento smartpointer).
                           // Se il canale di comunicazione è stato aperto (abbiamo quindi avuto
                           // un errore nella inizializzazione del drive/motor), il canale di 
                           // comunicazione verrà anche chiuso nella fase di deallocazione dell'oggetto
                           // SerialCommChannel
            // readyState = false; // non è necessaria questa istruzione 
                
            }
            catch(ElectricPowerException e){ //DOVRA' CATTURARE UN'ECCEZIONE DEFINITA DALL'UTENTE CHE
                     // STA AD INDICARE IL MANCATO SPEGNIMENTO DELL'ALIMENTAZIONE
                     // DEL MOTORE.
                     //.....
                     //.....
                     //.....
                e.badElectricPowerInfo();
                return -3;
            }
            return -4;
        }
        
        // A questo punto siamo certi che l'apertura del canale è andata a buon fine
        // ed i parametri del drive/motor sono stati inizializzati correttamente
        
        //readyState = true;
	//int state;
	//std::string desc;
	//this->getState(&state,desc);
	//DPRINT("ALEDEBUG state is %d (%s)",state,desc.c_str());
        
	return 0;
    }
   ERR("error parsing initialization string:\"%s\"",params.c_str());
   return -10;
}

int ActuatorTechnoSoft::deinit(){
    DPRINT("deinitializing");
    readyState=false;
    if (driver!=NULL) {
        delete driver;
    }
    return 0; 
}

int ActuatorTechnoSoft::moveRelativeMillimeters(double deltaMillimeters){
    DPRINT("moving relative %f mm",deltaMillimeters);
    
    // Calcolo argomento funzione moveRelativeSteps
    double deltaMicroSteps = round((N_ROUNDS*STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    long deltaMicroStepsL = deltaMicroSteps;
    if(driver->moveRelativeSteps(deltaMicroStepsL)<0)
        return -2;
    
    return 0;
}

int ActuatorTechnoSoft::moveAbsoluteMillimeters(double millimeters){ 
    
    // Calcolo argomento funzione moveAbsoluteSteps
    double nMicroSteps = round((N_ROUNDS*STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS);
    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    long nMicroStepsL = nMicroSteps;
    if(driver->moveAbsoluteSteps(nMicroStepsL)<0)    
        return -2;
    
    return 0;
}

int ActuatorTechnoSoft::getPosition(readingTypes mode, float* deltaPosition_mm){
     
    DPRINT("Position reading");

    if(mode==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(driver->getCounter(tposition)<0){
            DERR("getting counter");
            return -1;
        }
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        *deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(STEPS_PER_ROUNDS*CONST_MULT_TECHNOFT*N_ROUNDS);
    }
    else if(mode==READ_ENCODER){ // Lettura posizione per mezzo dell'encoder (Apos register)
        long aposition;
        
        if(driver->getEncoder(aposition)<0)
            return -2;
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        *deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS)/(N_ENCODER_LINES*N_ROUNDS);
    }
    return 0;
}

int ActuatorTechnoSoft::homing(homingType mode, std::string& descrErr){
    
    struct timeval structTimeEval;
    descrErr = "";
    
    if(mode==nativeHoming15){
        
        /*	Settings of homing procedure parameters */
        double high_speed = 10;		/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling] */
        double low_speed = 1.;		/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
        double acceleration = 0.3;	/* the acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
        double deceleration = 1.5;	/* the decceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
        long home_position = 1000;	/* the homing position [drive internal position units, encoder counts]  */
        
        /*	Set the homing parameters */
        
        /*	Set the acceleration rate for homing */
        if(driver->setFixedVariable("CACC", acceleration, descrErr)<0){
            return -1;
        }
        /*	Set the deceleration rate for homing */
        if(driver->setDecelerationParam(deceleration,descrErr)<0){
            return -2;
        }
        /*	Set the high speed for homing */
        if(driver->setFixedVariable("CSPD", high_speed,descrErr)<0){
            return -3;
        }
        /*	Set the low speed for homing */
        if(driver->setFixedVariable("HOMESPD", low_speed,descrErr)<0){
            return -4;
        }
        /*	Setup the home position at the end of the homing procedure */
        if(driver->setVariable("HOMEPOS", home_position,descrErr)<0) {
            return -5;
        }
        
        double currentTime_ms;
        WORD homing_done = 0;
        short indexRegSRL = 3; // see the constant value of REG_SRL in TML_lib.h
        
        /*	Call the homing procedure stored in the non-volatile memory of the drive */
        if(driver->executeTMLfunction("Homing15",descrErr)<0){
            return -6;
        }

        /*	Wait until the homing process is ended */
        gettimeofday(&structTimeEval, NULL);
        double iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        
        /*	Wait for homing procedure to end */
        while(homing_done == 0){
            /* Check the SRL.8 bit - Homing active flag */
            if((driver->getStatusOrErrorReg(indexRegSRL, homing_done, descrErr))<0){
                //Messa in sicurezza movimentazione:
                if(driver->abortNativeOperation(descrErr)<0){
                    return -7;
                }
                if(driver->stopMotion(descrErr)<0){ // Inseriamolo comunque questo comando..
                    return -8;
                }
                return -9;// Il sistema è stato messo in sicurezza
            }
            homing_done=((homing_done & 1<<8) == 0 ? 1 : 0);
            // ***** Quindi quando il bit 8 di REGSRL è uguale a 0, significa che l'operazione di Homing è terminata *********
            /*	If the homing procedure doesn't ends in timeo_homing_ms time then abort it */
            gettimeofday(&structTimeEval, NULL);
            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
            if(currentTime_ms - iniTime_ms >= timeo_homing_ms){
                if (homing_done != 0) {
                    return 0;
                }
                //Il tempo massimo di checking è scaduto.
                // Messa in sicurezza movimentazione
                if(driver->abortNativeOperation(descrErr)<0){
                    return -10;
                }
                if(driver->stopMotion(descrErr)<0){
                    return -11;
                }
                return -12; // Il tempo massimo di checking è scaduto, ma il sistema di movimentazione è stato cmq messo in sicurezza
            }
            usleep(1000); // Check each millisecond
        }//chiude while(homing_done == 0)
        return 0;
    } // chiude if(mode==nativeHoming15)
    
//    else if(mode==homing2){
//        /*	Movement parameters	*/
//	long position_mm = -100000000;		/* position command [drive internal position units, encoder counts] */
//	long home_position = 1000;		/* the homing position [drive internal position units, encoder counts] */
//	double high_speed = 10;			/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling]*/
//	double low_speed = 1.0;			/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
//	double acceleration = 0.6;		/* acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
//        
//        double iniTime_ms;
//        double currentTime_ms;
//        BOOL wait_event = FALSE; 
//        BOOL no_stop = FALSE; 
//        BOOL switchTransited = FALSE;
//        
//         
//        double tol_ms = 0.000000001;
//        double perc1=0.8;
//        double perc2=0.1;
//        double perc3=0.1;
//        // nota: perc1+perc2+perc3=1
//        
//        long cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
//        
//        /* Command a trapezoidal positioning to search the negative limit switch */
//   
//        //if(!TS_MoveRelative(position,high_speed, acceleration, NO_ADDITIVE,UPDATE_IMMEDIATE,FROM_REFERENCE)) 
//		//return FALSE;
//        
//        // ATTENZIONE: 
//        // Adesso position sarà inteso in mm
//        // ******** E' necessario settare i membri isAdditive,movement,referenceBase 
//        // al valore desiderato.************
//        
//        // position -> position_mm    
//        // high_speed -> membro speed OK
//        // acceleration -> member acceleration OK
//        // member isAdditive=0; 
//        // member movement = UPDATE_IMMEDIATE;
//        // member referenceBase = FROM_REFERENCE;
//        
//        if(moveRelativeMillimeters(position_mm)<0)
//            return -1;
//	
//        /*	Wait for the HIGH-LOW transition on negative limit switch */
//	if(driver->setEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, wait_event, no_stop)) 
//            return -2;
//        
//        gettimeofday(&structTimeEval, NULL);
//        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
//        currentTime_ms = iniTime_ms+tol_ms;
//       
//        while(!switchTransited && ((currentTime_ms - iniTime_ms) < timeo_ms*perc1)){
//            
//            if(!driver->checkEvent(switchTransited))
//                return -3;
//            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
//        }
//        
//        BOOL motionCompleted = FALSE;
//        if(switchTransited){
//            /*	Wait until the motor stops */
//            if(!driver->setEventOnMotionComplete(wait_event,no_stop)) 
//                return -4;
//            gettimeofday(&structTimeEval, NULL);
//            iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
//            currentTime_ms = iniTime_ms+tol_ms;
//            while(!motionCompleted && (currentTime_ms - iniTime_ms < timeo_ms*perc2)){
//                if(!driver->checkEvent(motionCompleted))
//                    return -5;
//            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
//            }
//        }
//        else 
//            return -50; // valore che scegliamo per denotare la scadenza del timeout
//        
//        if(motionCompleted){
//            /*	Read the captured position on limit switch transition */
//            if(!driver->getLVariable("CAPPOS", &cap_position)) 
//                return -6;
//            //printf(" The captured position is: %ld [drive internal position units]\n", cap_position);
//        }
//        else
//            return -50; // valore che scegliamo per denotare la scadenza del timeout
//
//        /*	Command an absolute positioning on the captured position */
//	if(moveAbsoluteMillimeters(cap_position)!=0); 
//            return -7;
//
//        /*	Wait for positioning to end */
//        
//	if(!driver->setEventOnMotionComplete(wait_event,no_stop)) 
//            return -8;
//        gettimeofday(&structTimeEval, NULL);
//        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
//        currentTime_ms = iniTime_ms+tol_ms;
//        
//        BOOL absoluteMotionCompleted = FALSE;
//        while(!absoluteMotionCompleted && (currentTime_ms - iniTime_ms < timeo_ms*perc3)){
//             if(!driver->checkEvent(absoluteMotionCompleted))
//                    return -9;
//            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
//        }
//        
//        /*	Set the home position */
//        if(absoluteMotionCompleted){
//            if(!driver->setPosition(home_position)) 
//            return -10;
//        }
//        else 
//            return -50; // valore che scegliamo per denotare la scadenza del timeout
//        //printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
//	//printf(" Homing procedure done!\n");
//	return 0;
//    }
//    else if(mode==defaultHoming){
//        
//        
//    }
//    return -50; // Indica che non ho indicato alcuna modalità corretta
}

int ActuatorTechnoSoft::getState(int* state, std::string& desc){
    
    *state = ACTUATOR_UNKNOWN_STATUS;
    // *state = ACTUATOR_READY;
    //return 0;
    desc.assign("Unknown state");
    int stCode=0;
    
    DPRINT("Getting state of the actuator");
    
    WORD contentRegSRH; // remember typedef uint16_t WORD; 
    WORD contentRegSRL;
  
    short indexRegSRH = 4; // see constant REG_SRH in TML_lib.h
    
    if((driver->getStatusOrErrorReg(indexRegSRH, contentRegSRH, desc))<0){
        ERR("Reading state error: %s",desc.c_str());
        return -1;
    }
    
    if(readyState)
        stCode|=ACTUATOR_READY;
    
    // Analysis of the register content SRH
    bool overPositionState = false;
    for(WORD i=1; i<=4; i++){
        if(contentRegSRH & 1 << i){ 
            stCode |= ACTUATOR_OVER_POSITION_TRIGGER;
            overPositionState = true;
        }
    }
    if(overPositionState)
        desc.assign("Position trigger. "); // **************DA QUI IN POI LA STRINGA DOVRÀ ESSERE CONCATENATA
                                         // con il contenuto corrente **************
    if(contentRegSRH & (1<<5)){
        stCode |= ACTUATOR_AUTORUN_ENABLED;
        desc+="Auto run mode. ";
    }
    if(contentRegSRH & (1<<6)){
        stCode |= ACTUATOR_LSP_EVENT_INTERRUPUT;
        desc+="Limit switch positive event/interrupt. ";
    }
    if(contentRegSRH & (1<<7)){
        stCode |= ACTUATOR_LSN_EVENT_INTERRUPT;
        desc+="Limit switch negative event/interrupt. ";
    }
    if(contentRegSRH & (1<<12)){
        stCode |= ACTUATOR_IN_GEAR;
        desc+="Gear ratio in electronic gearing mode. ";
    }
    if(contentRegSRH & (1<<14)){
        stCode |= ACTUATOR_IN_CAM;
        desc+="Reference position in absolute electronic camming mode. ";
    }
    if(contentRegSRH & (1<<15)){
        stCode |= ACTUATOR_FAULT;
        desc+="Fault status. ";
    }
    
    short indexRegSRL = 3; // see constant REG_SRH in TML_lib.h
    if((driver->getStatusOrErrorReg(indexRegSRL, contentRegSRL, desc))<0){
        ERR("Reading state error: %s",desc.c_str());
        return -2;
    }
    //  Analysis of the register content SRL
    if(contentRegSRL & (1<<10)){
        stCode |= ACTUATOR_MOTION_COMPLETED;
        desc+="Actuator motion completed. ";
    }
    if(contentRegSRL & (1<<15)){
        stCode |= ACTUATOR_POWER_SUPPLIED;
        desc += "Electrical power supplied.";
    }
    *state = stCode;
    return 0;
}

int ActuatorTechnoSoft::getAlarms(uint64_t*alrm){
    
    uint64_t stCode=0;
    
    DPRINT("Getting alarms of the actuator");
    
    WORD contentRegMER; // remember typedef uint16_t WORD;
    WORD contentRegSRH;
    std::string desc; // *************IMPORTANTE: QUESTA STRINGA DOVRÀ ESSERE POI ARGOMENTO 
                       // DELLA FUNZIONE getAlarms. Bisognerà quindi cambiare in seguito il prototipo.
    
    short indexRegMER = 5; // see constant REG_MER in TML_lib.h
    if(driver->getStatusOrErrorReg(indexRegMER, contentRegMER, desc)<0){
        DERR("Reading alarms error: %s",desc.c_str());
        return -1;
    }
    
    short indexRegSRH = 4; // see constant REG_SRH in TML_lib.h
    if(driver->getStatusOrErrorReg(indexRegSRH, contentRegSRH, desc)<0){
        DERR("Reading alarms error: %s",desc.c_str());
        return -2;
    }
    
    WORD base2 = 2;
    // Analysis of the register content REG_MER
    for(WORD i=0; i<sizeof(WORD)*8; i++){
        if(contentRegMER & ((WORD)(base2^i))){ 
            if(i==0){
                stCode|=ACTUATOR_CANBUS_ERROR; // IMPORTANTE: ACTUATOR_CANBUS_ERROR è di tipo int (32 bit)
                                               // Nell'operazione di OR logico, automaticamente il contenuto
                                               // a destra dell'uguale viene prima memorizzato in una locazione
                                               // a 64 bit di tipo unsigned cosicché si possa fare l'OR logico 
                                               // bit a bit con la variabile a primo membro
                                               // In corrispondenza di questo errore accendo il bit 0 di *alarm
                desc.assign("CAN bus error. ");
            }
            else if(i==1){
                stCode|=ACTUATOR_SHORT_CIRCUIT; // In corrispondenza di questo errore accendo il bit 1 di *alarm
                desc+="Short circuit. ";
            }
            else if(i==2){
                stCode|=ACTUATOR_INVALID_SETUP_DATA;
                desc+= "Invalid setup data. ";
            }
            else if(i==3){
                stCode|=ACTUATOR_CONTROL_ERROR;
                desc+= "Control error. ";
            }
            else if(i==4){
                stCode|=ACTUATOR_SERIAL_COMM_ERROR;
                desc+= "Communication error. ";
            }
            else if(i==5){
                stCode|=ACTUATOR_HALL_SENSOR_MISSING;
                desc+= "Hall sensor missing / Resolver error / BiSS error / Position wrap around error. ";
            }
            else if(i==6){
                stCode|=ACTUATOR_LSP_LIMIT_ACTIVE;
                desc+="Positive limit switch active. ";
            }
            else if(i==7){
                stCode|=ACTUATOR_LSN_LIMIT_ACTIVE;
                desc+="Negative limit switch active. ";
            }
            else if(i==8){
                stCode|=ACTUATOR_OVER_CURRENT;
                desc+="Over current error. ";
            }
            else if(i==9){
                stCode|=ACTUATOR_I2T;
                desc+="I2T protection error. ";
            }
            else if(i==10){
                stCode|=ACTUATOR_OVERTEMP_MOTOR;
                desc+="Motor over temperature error. ";
            }
            else if(i==11){
                stCode|=ACTUATOR_OVERTEMP_DRIVE;
                desc+="Drive over temperature error. ";
            }
            else if(i==12){
                stCode|=ACTUATOR_OVERVOLTAGE;
                desc+="Over voltage error. ";
            }
            else if(i==13){
                stCode|=ACTUATOR_UNDERVOLTAGE;
                desc+="Under voltage error. ";
            }
            else if(i==14){
                stCode|=ACTUATOR_COMMANDERROR;
                desc+="Command error. ";
            }
        }
    }
    // Analysis of the register content REG_SRH
    if(contentRegSRH & ((WORD)(base2^10))){
        stCode|=(uint64_t)ACTUATOR_I2T_WARNING_MOTOR;
        desc+="Motor I2T protection warning. ";
    }
    if(contentRegSRH & ((WORD)(base2^11))){
        stCode|=(uint64_t)ACTUATOR_I2T_WARNING_DRIVE;
        desc+="Drive I2T protection warning";
    }
    
    *alrm = stCode;
    return 0;
}
        
//#if 0
//BOOL ActuatorTechnoSoft::homing(int minutes,std::string& mode){
//    
//    if(mode.compare("mode_1")==0){
//    /*	Settings of homing parameters for this example */
//	double high_speed = 10;		/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling] */
//	double low_speed = 1.;		/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
//	double acceleration = 0.3;	/* the acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
//	double deceleration = 1.5;	/* the decceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
//	long home_position = 1000;	/* the homing position [drive internal position units, encoder counts]  */
//	time_t long_time;
//	struct tm *ini_time;
//	struct tm *new_time;
//	int iMinute, nMinute;
//	WORD homing_done = 0;
//        
///*	Set the homing parameters */
///*	--------------------------------------------------------------------*/
///*	Set the acceleration rate for homing */
//	if(!driver->setFixedVariable("CACC", acceleration)) 
//		return -1;		
///*	Set the deceleration rate for homing */
//	if(!driver->setDecelerationParam(deceleration)) 
//		return -2;	
///*	Set the high speed for homing */
//	if(!driver->setFixedVariable("CSPD", high_speed)) 
//		return -3;		
///*	Set the low speed for homing */
//	if(!driver->setFixedVariable("HOMESPD", low_speed)) 
//		return -4;	
///*	Setup the home position at the end of the homing procedure */
//	if(!driver->setVariable("HOMEPOS", home_position)) 
//		return -5;	
///*	--------------------------------------------------------------------*/
//
///*	Call the homing procedure stored in the non-volatile memory of the drive */
//	/*if(!driver->executeTMLfunction("Homing15"))
//		return -6;
//*/
///*	Wait until the homing process is ended */
//	time( &long_time );
//	ini_time = localtime( &long_time );
//	iMinute = ini_time->tm_min;
//	
///*	Wait for homing procedure to end */	
//	while(homing_done == 0){
///*
// * Check the SRL.8 bit - Homing active flag*/
//
//		if(!TS_ReadStatus(REG_SRL, &homing_done)){
//                    /*	Cancel the homing procedure   */
//                    if(!driver->abortNativeOperation()) 
//                        return -7;
//                    /*	Stop the motor */
//                    if(!driver->stopMotion()) 
//			return -8;
//                    return -9;
//                }
//
//		homing_done=((homing_done & 1<<8) == 0 ? 1 : 0);
//
///*	If the homing procedure doesn't ends in MINUTES time then abort it */
//		time( &long_time );
//		new_time = localtime( &long_time );
//		nMinute = new_time->tm_min;
//		if(nMinute - iMinute >= minutes){
//			break;
//		}
//                usleep(1000); // Check each millisecond
//	}
//        /*if (homing_done != 0){
//		printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
//		printf(" Homing procedure done!\n");
//	}
//         * */
//        if(homing_done == 0){
//        /*	Cancel the homing procedure	*/
//		if(!driver->abortNativeOperation()) 
//			return -10;
//
//        /*	Stop the motor */
//		if(!driver->stopMotion()) 
//			return -11;
//
//		//printf(" After %d [min] the homing procedure was not finished!\n", minutes);
//                
//                return -12;
//	}	
//	return 0;
//    }
//    else if(mode.compare("mode_2")==0){
//        /*	Movement parameters	*/
//	long position = -100000000;		/* position command [drive internal position units, encoder counts] */
//	long home_position = 1000;		/* the homing position [drive internal position units, encoder counts] */
//	long cap_position = 0;			/* the position captures at HIGH-LOW transition of negative limit switch */
//	//double high_speed = 10;			/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling]*/
//	double low_speed = 1.0;			/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
//	//double acceleration = 0.6;		/* acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
//        
///*	Command a trapezoidal positioning to search the negative limit switch */
//   
//        //if(!TS_MoveRelative(position, speed, acceleration, isAdditive,movement,referenceBase)) 
//		//return FALSE;
//        
//        // ATTENZIONE: 
//        // Adesso position sarà inteso in mm
//        // ******** E' necessario settare i membri isAdditive,movement,referenceBase 
//        // al valore desiderato.************
//        
//        if(!moveRelativeMillimeters(position))
//            return -15;
//	
///*	Wait for the HIGH-LOW transition on negative limit switch */
//	if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, WAIT_EVENT, NO_STOP)) 
//		return FALSE;
//
///*	Wait until the motor stops */
//	if(!TS_SetEventOnMotionComplete(WAIT_EVENT,NO_STOP)) 
//		return FALSE;
//
///*	Read the captured position on limit switch transition */
//	if(!TS_GetLongVariable("CAPPOS", &cap_position)) return FALSE;
//	printf(" The captured position is: %ld [drive internal position units]\n", cap_position);
//
///*	Command an absolute positioning on the captured position */
//	if(!TS_MoveAbsolute(cap_position, low_speed, acceleration, UPDATE_IMMEDIATE, FROM_REFERENCE)) 
//		return FALSE;
//
///*	Wait for positioning to end */
//	if(!TS_SetEventOnMotionComplete(WAIT_EVENT,NO_STOP)) 
//		return FALSE;
//
///*	Set the home position */
//	if(!TS_SetPosition(home_position)) 
//		return FALSE;
//	
//	printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
//	printf(" Homing procedure done!\n");
//
//	return TRUE;
//    }
//    return 0;
//}

//int getSWVersion(std::string& versionFm){
//    
//    /*	Read the firmware version programmed on the drive */
//    char version[10];
//    if (!MSK_GetDriveVersion(version)) 
//	return -1;
//    //printf("\n The drive is programmed with %s firmware.\n\n", FW_version);
//    versionFm.assign(version);        
//    
//    return 0;
//}



//#endif
