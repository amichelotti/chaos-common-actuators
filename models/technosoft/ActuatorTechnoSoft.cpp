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
        int val;
        if((val=driver->init(conf_path,atoi(axid.c_str())))<0){
            printf("Iipologia di errore in fase di inizializzazione %d \n",val);
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
        
        readyState = true;
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
    double deltaMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    //long deltaMicroStepsL = deltaMicroSteps;
    if(driver->moveRelativeSteps((long)deltaMicroSteps)<0)
        return -2;
    
    return 0;
}

int ActuatorTechnoSoft::moveRelativeMillimetersHoming(double deltaMillimeters){
    
    DPRINT("moving relative %f mm",deltaMillimeters);
    
    // Calcolo argomento funzione moveRelativeSteps
    double deltaMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX){ // solo per adesso e necessario questo filtro..
        printf("Out of range\n");
        return -1;
    }
    
    //long deltaMicroStepsL = deltaMicroSteps;
    if(driver->moveRelativeStepsHoming((long)deltaMicroSteps)<0){
        return -2;
    }
    
    return 0;
}


int ActuatorTechnoSoft::setTrapezoidalProfile(double speed, double acceleration, bool isAdditive, int32_t movement, int32_t referenceBase){
    
    if(driver->setSpeed(speed)<0){
        return -1;
    }
    if(driver->setAcceleration(acceleration)<0){
        return -2;
    }
    if(driver->setAdditive(isAdditive)<0){
        return -3;
    }
    if(driver->setMovement(movement)<0){
        return -4;
    }
    if(driver->setReferenceBase(referenceBase)<0){
        return -5;
    }
    return 0;
}

int ActuatorTechnoSoft::setSpeed(double speed){
    if(driver->setSpeed(speed)<0){
        return -1;
    }
    return 0;
    
}

int ActuatorTechnoSoft::setAcceleration(double acceleration){
    if(driver->setAcceleration(acceleration)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setAdditive(bool isAdditive){
    if(driver->setAdditive((int)isAdditive)<0){
        return -1;
    }
    return 0;
                    
}

int ActuatorTechnoSoft::setMovement(int32_t movement){
    if(driver->setMovement((short)movement)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setReferenceBase(int32_t referenceBase){
    if(driver->setReferenceBase((short)referenceBase)<0){
        return -1;
    }                
    return 0;
}

int ActuatorTechnoSoft::moveAbsoluteMillimeters(double millimeters){ 
    
    // Calcolo argomento funzione moveAbsoluteSteps
    double nMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    printf("nMicroSteps=%f\n",nMicroSteps);
    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    //long nMicroStepsL = nMicroSteps;
    if(driver->moveAbsoluteSteps((long)nMicroSteps)<0)    
        return -2;
    
    return 0;
}

int ActuatorTechnoSoft::moveAbsoluteMillimetersHoming(double millimeters){ 
    
    // Calcolo argomento funzione moveAbsoluteSteps
    double nMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    printf("nMicroSteps=%f\n",nMicroSteps);
    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX) // solo per adesso e necessario questo filtro..
        return -1;
    
    //long nMicroStepsL = nMicroSteps;
    if(driver->moveAbsoluteStepsHoming((long)nMicroSteps)<0){    
        return -2;
    }
    
    return 0;
}

int ActuatorTechnoSoft::getPosition(readingTypes mode, double* deltaPosition_mm){
     
    DPRINT("Position reading");

    if(mode==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(driver->getCounter(tposition)<0){
            DERR("getting counter");
            return -1;
        }
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        *deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*N_ROUNDS_DEFAULT);
    }
    else if(mode==READ_ENCODER){ // Lettura posizione per mezzo dell'encoder (Apos register)
        long aposition;
        
        if(driver->getEncoder(aposition)<0)
            return -2;
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        *deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(N_ENCODER_LINES_DEFAULT*N_ROUNDS_DEFAULT);
    }
    return 0;
}

int ActuatorTechnoSoft::homing(homingType mode){
    
    struct timeval structTimeEval;
    long currentTime_ms;
    double iniTime_ms;
    
    if(mode==nativeHoming15){
    /*	Settings of homing parameters for this example */
	double high_speed = 10;		/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling] */
	double low_speed = 1.;		/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
	double acceleration = 0.3;	/* the acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
	double deceleration = 1.5;	/* the decceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
	long home_position = 1000;	/* the homing position [drive internal position units, encoder counts]  */
        
/*	Set the homing parameters */
/*	--------------------------------------------------------------------*/
/*	Set the acceleration rate for homing */
	if(driver->setFixedVariable("CACC", acceleration)<0) 
		return -1;		
/*	Set the deceleration rate for homing */
	if(driver->setDecelerationParam(deceleration)<0) 
		return -2;	
/*	Set the high speed for homing */
	if(driver->setFixedVariable("CSPD", high_speed)<0) 
		return -3;		
/*	Set the low speed for homing */
	if(driver->setFixedVariable("HOMESPD", low_speed)<0) 
		return -4;	
/*	Setup the home position at the end of the homing procedure */
	if(driver->setVariable("HOMEPOS", home_position)<0) 
		return -5;	
/*	--------------------------------------------------------------------*/

        std::string str="Homing15";
/*	Call the homing procedure stored in the non-volatile memory of the drive */
	if(driver->executeTMLfunction(str)<0) 
		return -6;

/*	Wait until the homing process is ended */
        //long currentTime_ms;
        uint16_t homing_done;
        str.assign("");
        
        gettimeofday(&structTimeEval, NULL);
        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        /*	Wait for homing procedure to end */	
	while(homing_done == 0){
/*	Check the SRL.8 bit - Homing active flag*/
		if(driver->getStatusOrErrorReg(3, homing_done,str)<0){ 
                    /*	Cancel the homing procedure   */
                    if(driver->abortNativeOperation()<0) 
                        return -7; // HARD ERROR
                    /*	Stop the motor */
                    if(driver->stopMotion()<0) 
			return -8; // HARD ERROR
                    return -9;
                }
		homing_done=((homing_done & 1<<8) == 0 ? 1 : 0);

/*	If the homing procedure doesn't ends in MINUTES time then abort it */
                gettimeofday(&structTimeEval, NULL);
                currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
		if(currentTime_ms - iniTime_ms >= timeo_homing_ms){
			break;
		}
                usleep(1000); // Check each millisecond
	}//chiude while(homing_done == 0)
        
        if(homing_done == 0){ // Time out scaduto
        /*	Cancel the homing procedure	*/
		if(driver->abortNativeOperation()<0) 
			return -10;

        /*	Stop the motor */
		if(driver->stopMotion()<0) 
			return -11;
                
                return -12;
	}
      
        // Reset encoder e counter
        if(driver->resetEncoder()<0)
            return -13;
        if(driver->resetCounter()<0)
            return -14;
        
	return 0;
    }
    else if(mode==homing2){
        DPRINT("************** Operazione di homing partita. Durera al massimo %lu ms**************", timeo_homing_ms);
        
        /*     Movement parameters     */
	//long position_mm = -1000;		/* position command [drive internal position units, encoder counts] */
	long home_position = 1000;		/* the homing position [drive internal position units, encoder counts] */
	double high_speed = 10;			/* the homing travel speed [drive internal speed units, encoder counts/slow loop sampling]*/
	                                        /* utilizzata in moveRelative*/
        
        double low_speed = 1.0;			/* the homing brake speed [drive internal speed units, encoder counts/slow loop sampling] */
	                                        /* Utilizzata in moveAbsolute, riposizionamento*/
        
        double acceleration = 0.6;		/* acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2] */
                                                /*Utilizzata sia in moveRelative che in moveAbsolute*/
        //double iniTime_ms;
        //double currentTime_ms;
//        BOOL wait_event = FALSE; 
//        BOOL no_stop = FALSE; 
//        BOOL switchTransited = FALSE;
        int wait_event=0;
        int no_stop=0;
        int switchTransited=0;
         
        double tol_ms = 0.000000001;
        double perc1=0.9;
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
        
        if(driver->moveVelocityHoming()<0){
            return -1;
        }
	
        DPRINT("************** Wait for the HIGH-LOW transition on negative limit switch **************");
        /*	Wait for the HIGH-LOW transition on negative limit switch */
	if(driver->setEventOnLimitSwitch()<0){
            if(driver->stopMotion()<0){
                return -2;
            }
            return -3;
        }
        
        gettimeofday(&structTimeEval, NULL);
        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        currentTime_ms = iniTime_ms+tol_ms;
        double timeLimit = timeo_homing_ms*perc1;
        
        DPRINT("************** Time interval available for waiting the HIGH-LOW transition on negative limit switch: %f **************",timeLimit);
        while(!switchTransited && ((currentTime_ms - iniTime_ms) <= timeLimit)){
            if(driver->checkEvent(switchTransited)<0){
                if(driver->stopMotion()<0){
                    return -4;
                }    
                return -5;
            }  
            gettimeofday(&structTimeEval, NULL);
            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
            usleep(1000);
        }
        int motionCompleted = 0;
        if(switchTransited){
            DPRINT("************** Negative limit switch transited **************");
            /*	Wait until the motor stops */
            DPRINT("************** Wait until the motor stops**************");
            if(driver->setEventOnMotionComplete()<0){ 
                if(driver->stopMotion()<0){
                    return -6;
                }
                return -7;
            }    
            gettimeofday(&structTimeEval, NULL);
            iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
            currentTime_ms = iniTime_ms+tol_ms;
            timeLimit = timeo_homing_ms*perc2;
            DPRINT("************** Time interval available for waiting the motor stops: %f **************",timeLimit);
            while(!motionCompleted && ((currentTime_ms - iniTime_ms) <= timeLimit)){
                if(driver->checkEvent(motionCompleted)<0){
                    if(driver->stopMotion()<0){
                        return -8;
                    }
                    return -9;
                }
                gettimeofday(&structTimeEval, NULL);
                currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
                usleep(1000);
            }
        }
        else{
            DPRINT("************** Negative limit switch is not transited in the specified time interval %f ms**************",currentTime_ms - iniTime_ms);
            if(driver->stopMotion()<0){
                return -11;
            }
            return -12; // valore che scegliamo per denotare la scadenza del timeout*perc1 senza che sono riuscito 
                       // a rilevare la transizione dello swicth
        }
        if(motionCompleted){ // In questo caso il motore è fermo
            DPRINT("************** Motion completed in %f ms from switch transition **************",currentTime_ms - iniTime_ms);
            /*	Read the captured position on limit switch transition */
            DPRINT("************** Read the captured position on limit switch transition**************");
            std::string cappos = "CAPPOS";
            if(driver->getLVariable(cappos, cap_position)<0){ 
//                if(driver->stopMotion()<0){
//                    return -13;
//                }
                return -14;
            }
            DPRINT("************** the captured position on limit switch transition is %ld [drive internal position units]**************",cap_position);
            //printf(" The captured position is: %ld [drive internal position units]\n", cap_position);
        }
        else{
            DPRINT("************** The motion is not completed in the specified time interval %f ms**************",currentTime_ms - iniTime_ms);
            if(driver->stopMotion()<0){
                return -15;
            }
            return -16; // valore che scegliamo per denotare la scadenza del timeout nell'attendere lo
                        // stop del motore una volta che la transizione dello switch è avvenuta
        }

        DPRINT("************** Command an absolute positioning on the captured position **************");
        /*	Command an absolute positioning on the captured position */
        if(driver->moveAbsoluteStepsHoming(cap_position)<0){    
            return -17;
        }

        /*	Wait for positioning to end */
        DPRINT("************** Wait for positioning to end  **************");
	if(driver->setEventOnMotionComplete()<0){
            if(driver->stopMotion()<0){
                return -18;
            }
            return -19;
        }   
            
        gettimeofday(&structTimeEval, NULL);
        iniTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
        currentTime_ms = iniTime_ms+tol_ms;
        timeLimit = timeo_homing_ms*perc3;
        
        int count=0;
        
        int absoluteMotionCompleted = 0;
        
        DPRINT("************** Time interval available for waiting positioning to end: %f **************",timeLimit);
        while(!absoluteMotionCompleted && ((currentTime_ms - iniTime_ms) < timeLimit)){
            if(driver->checkEvent(absoluteMotionCompleted)<0){
                if(driver->stopMotion()<0){
                    return -20;
                }
                return -21;
            }
            gettimeofday(&structTimeEval, NULL);
            currentTime_ms = structTimeEval.tv_sec * 1000 + structTimeEval.tv_usec / 1000;
            usleep(1000);
            
            count++;
        }
        
        DPRINT("************** Motor has been repositioned after checked %d times the motion completed event, after %f ms **************", count,currentTime_ms - iniTime_ms);
//        
        /*	Set the home position */
        DPRINT("************** Set the home position  **************");
        if(absoluteMotionCompleted){
//            if(driver->setPosition(home_position)<0){ 
//                return -22;
//            }
            // Reset encoder e counter
            DPRINT("************** Reset encoder e counter **************");
            if(driver->resetEncoder()<0){
                return -23;
            }
            if(driver->resetCounter()<0){
                return -24;
            }
        }
        else{
            DPRINT("************** The repositioning procedure is not finished in the time interval of %f ms  **************",timeLimit);
            return -25; // valore che scegliamo per denotare la scadenza della terza parte del timeout
        }
////        //printf(" The motor position is set to %ld [position internal units]!\n\n", home_position);
////	//printf(" Homing procedure done!\n");
//        
//        // Reset encoder e counter
//        DPRINT("************** Reset encoder e counter **************");
//        if(driver->resetEncoder()<0)
//            return -24;
//        if(driver->resetCounter()<0)
//            return -25;
        DPRINT("************** Operazione di homing terminata**************");
	return 0;
    }
    else if(mode==defaultHoming){
        //..
        //..
        DPRINT("************** Operazione di homing terminata**************");
        return 0;
    }
    return -50;
}

int ActuatorTechnoSoft::getState(int* state, std::string& descStr){

    DPRINT("Getting state of the actuator. ");

    *state  = ACTUATOR_UNKNOWN_STATUS;
    descStr.assign("");

    int stCode=0;

    uint16_t contentRegSRH; // remember typedef uint16_t WORD;
    uint16_t contentRegSRL;

    short indexReg = 4; // see constant REG_SRH in TML_lib.h
    if((driver->getStatusOrErrorReg(indexReg, contentRegSRH, descStr))<0){
        ERR("Reading state error: %s",descStr.c_str());
        return -1;
    }
    indexReg = 3; // see constant REG_SRL in TML_lib.h
    if((driver->getStatusOrErrorReg(indexReg, contentRegSRL, descStr))<0){
        ERR("Reading state error: %s",descStr.c_str());
        return -2;
    }

    if(readyState){ // readyState = true se la procedura di inizializzazione è andata a buon fine. Accendo il primo bit
        stCode|=ACTUATOR_READY;
        descStr=descStr+"Ready. ";
    }

    // Analysis of the register content SRH (bit 1,2,3,4)
    bool overPositionTrigger = false;
    for(int i=1; i<=4; i++){
        if(contentRegSRH & ((uint16_t)1 << i)){
            stCode |= ACTUATOR_OVER_POSITION_TRIGGER; // accendo il secondo bit
            overPositionTrigger = true;
        }
    }

    if(overPositionTrigger){
        //desc.assign("Position trigger. "); // **************DA QUI IN POI LA STRINGA DOVRÀ ESSERE CONCATENATA
        descStr=descStr+"Over position trigger. ";
    }
    // con il contenuto corrente **************
    if(contentRegSRH & ((uint16_t)1<<5)){
        stCode |= ACTUATOR_AUTORUN_ENABLED;
        descStr+="Auto run mode. ";
    }
    if(contentRegSRH & ((uint16_t)1<<6)){
        stCode |= ACTUATOR_LSP_EVENT_INTERRUPUT;
        descStr+="Limit switch positive event/interrupt. ";
    }
    if(contentRegSRH & ((uint16_t)1<<7)){
        stCode |= ACTUATOR_LSN_EVENT_INTERRUPT;
        descStr+="Limit switch negative event/interrupt. ";
    }
    if(contentRegSRH & ((uint16_t)1<<12)){
        stCode |= ACTUATOR_IN_GEAR;
        descStr+="Gear ratio in electronic gearing mode. ";
    }
    if(contentRegSRH & ((uint16_t)1<<14)){
        stCode |= ACTUATOR_IN_CAM;
        descStr+="Reference position in absolute electronic camming mode. ";
    }
    if(contentRegSRH & ((uint16_t)1<<15)){
        stCode |= ACTUATOR_FAULT;
        descStr+="Fault status. ";
    }

    //  Analysis of the register content SRL
    if(contentRegSRL & ((uint16_t)1<<10)){
        stCode |= ACTUATOR_MOTION_COMPLETED;
        descStr+="Actuator motion completed. ";
    }
    if(contentRegSRL & ((uint16_t)1<<15)){
        stCode |= ACTUATOR_POWER_SUPPLIED;
        descStr += "Electrical power supplied.";
    }
    *state = stCode;
    return 0;
}

int ActuatorTechnoSoft::getAlarms(uint64_t* alrm, std::string& descStr){

    * alrm = ACTUATOR_NO_ALARMS_DETECTED;
    descStr.assign("");

    int stCode=0;

    DPRINT("Getting alarms of the actuator");

    uint16_t contentRegMER; // remember typedef uint16_t WORD;
    uint16_t contentRegSRH;

    short indexRegMER = 5; // see constant REG_MER in TML_lib.h
    if(driver->getStatusOrErrorReg(indexRegMER, contentRegMER, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        return -1;
    }

    short indexRegSRH = 4; // see constant REG_SRH in TML_lib.h
    if(driver->getStatusOrErrorReg(indexRegSRH, contentRegSRH, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        return -2;
    }

    // ***********************RIPRENDERE DA QUI*********************
    //WORD base2 = 2;
    // Analysis of the register content REG_MER
    // (WORD)(base2^i)
    for(uint16_t i=0; i<sizeof(uint16_t)*8; i++){
        if(contentRegMER & ((uint16_t)1<<i)){ // se il bit i-esimo di REG_MER è 1, i=0,1,...,15
            if(i==0){
                stCode|=ACTUATOR_CANBUS_ERROR; // IMPORTANTE: ACTUATOR_CANBUS_ERROR è di tipo int (32 bit)
                // Nell'operazione di OR logico, automaticamente il contenuto
                // a destra dell'uguale viene prima memorizzato in una locazione
                // a 64 bit di tipo unsigned cosicché si possa fare l'OR logico
                // bit a bit con la variabile a primo membro
                // In corrispondenza di questo errore accendo il bit 0 di *alarm
                //desc.assign("CAN bus error. ");
                descStr+="CAN bus error. ";
            }
            else if(i==1){
                stCode|=ACTUATOR_SHORT_CIRCUIT; // In corrispondenza di questo errore accendo il bit 1 di *alarm
                descStr+="Short circuit. ";
            }
            else if(i==2){
                stCode|=ACTUATOR_INVALID_SETUP_DATA;
                descStr+= "Invalid setup data. ";
            }
            else if(i==3){
                stCode|=ACTUATOR_CONTROL_ERROR;
                descStr+= "Control error. ";
            }
            else if(i==4){
                stCode|=ACTUATOR_SERIAL_COMM_ERROR;
                descStr+= "Communication error. ";
            }
            else if(i==5){
                stCode|=ACTUATOR_HALL_SENSOR_MISSING;
                descStr+= "Hall sensor missing / Resolver error / BiSS error / Position wrap around error. ";
            }
            else if(i==6){
                stCode|=ACTUATOR_LSP_LIMIT_ACTIVE;
                descStr+="Positive limit switch active. ";
            }
            else if(i==7){
                stCode|=ACTUATOR_LSN_LIMIT_ACTIVE;
                descStr+="Negative limit switch active. ";
            }
            else if(i==8){
                stCode|=ACTUATOR_OVER_CURRENT;
                descStr+="Over current error. ";
            }
            else if(i==9){
                stCode|=ACTUATOR_I2T;
                descStr+="I2T protection error. ";
            }
            else if(i==10){
                stCode|=ACTUATOR_OVERTEMP_MOTOR;
                descStr+="Motor over temperature error. ";
            }
            else if(i==11){
                stCode|=ACTUATOR_OVERTEMP_DRIVE;
                descStr+="Drive over temperature error. ";
            }
            else if(i==12){
                stCode|=ACTUATOR_OVERVOLTAGE;
                descStr+="Over voltage error. ";
            }
            else if(i==13){
                stCode|=ACTUATOR_UNDERVOLTAGE;
                descStr+="Under voltage error. ";
            }
            else if(i==14){
                stCode|=ACTUATOR_COMMANDERROR;
                descStr+="Command error. ";
            }
        }// chiudo if(contentRegMER & ((WORD)(base2^i)))
    } // chiudo for(WORD i=0; i<sizeof(WORD)*8; i++)

    // Analysis of the register content REG_SRH
    if(contentRegSRH & ((uint16_t)1<<10)){
        stCode|=ACTUATOR_I2T_WARNING_MOTOR;
        descStr+="Motor I2T protection warning. ";
    }
    if(contentRegSRH & ((uint16_t)1<<11)){
        stCode|=ACTUATOR_I2T_WARNING_DRIVE;
        descStr
                +="Drive I2T protection warning";
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
