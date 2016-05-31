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
static const boost::regex driver_match("([\\w\\/]+),(\\w+),(.+),(\\d+)"); //ATTENZIONE: DEVE ESSERE GESTITA LA QUINTA ESPRESSIONE REGOLARE

ActuatorTechnoSoft::ActuatorTechnoSoft(){
    driver=NULL;
    readyState=false;
}

ActuatorTechnoSoft::~ActuatorTechnoSoft(){
    deinit();
    DPRINT("Deallocazione oggetto actuator TechnSoft");
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
        std::string straxid=match[4];
        int axid = atoi(straxid.c_str());
        DPRINT("initializing \"%s\" dev:\"%s\" conf path:\"%s\"",dev_name.c_str(),dev.c_str(),conf_path.c_str());
        
        //ATTENZIONE: DEVE ESSERE GESTITE LE SEGUENTI ESPRESSIONI REGOLARI.
        // IL CONTROLLO DEI VALORI APPARTENENTI AL RANGE DI AMMISSIBILITA' DOVRA' ESSERE EFFETTUATO A QUESTO PUNTO
        try{
            driver = new TechnoSoftLowDriver(dev,dev_name);
            // ATTENZIONE: IL COSTRUTTORE TechnoSoftLowDriver DOVRÀ IN SEGUITO 
            // GENERARE UNA ECCEZIONE nel caso l'oggetto SerialCommChannelTechnosoft 
            // non venga allocato.
            // L'istruzione precedente dovrà essere invocata facendo uso di un blocco 
            // try/catch. Nel caso l'eccezione venga catturata dovrà essere restituito
            // un numero negativo
        }
        catch(std::bad_alloc& e){ // Eccezione derivante dal fatto che l'oggetto TechnoSoftLowDriver oppure
                                  // l'oggetto canale di comunicazione non è stato possibile allocarlo.
                                  
            ERR("## cannot create TechnoSoftLowDriver or channel object");
            if(driver!=NULL){ 
                delete driver;
            }
            
            //readyState = false; //non è necessari questa assegnazione
            return -1;
        }
        
        // Inizializzazione oggetto TechnoSoft low driver costruito
        
        int val;
        if((val=driver->init(conf_path,axid))<0){
            ERR("****************Iipologia di errore in fase di inizializzazione dell'oggetto technsoft low driver %d",val);
            // Deallocazione oggetto canale ()
            try{
                delete driver; // Nota importante: l'indirizzo del canale 
                           // non è stato ancora memorizzato nella mappa. Essendo l'oggetto
                           // canale puntato da un solo oggetto TechnoSOftLowDriver,
                           // anche l'oggetto canale sarà deallocato (funzionamento smartpointer).
                           // Se il canale di comunicazione è stato aperto (abbiamo quindi avuto
                           // un errore nella inizializzazione del drive/motor), il canale di 
                           // comunicazione verrà anche chiuso nella fase di deallocazione dell'oggetto
                           // SerialCommChannel
            // readyState = false; // non è necessaria questa istruzione
                driver = NULL;
                
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
            catch(StopMotionException e){ //DOVRA' CATTURARE UN'ECCEZIONE DEFINITA DALL'UTENTE CHE
                     // STA AD INDICARE IL MANCATO SPEGNIMENTO DELL'ALIMENTAZIONE
                     // DEL MOTORE.
                     //.....
                     //.....
                     //.....
                e.badStopMotionInfo();
                return -4;
            }
            
            return -5;
        }
        
        // A questo punto siamo certi che l'apertura del canale è andata a buon fine
        // ed i parametri del drive/motor sono stati inizializzati correttamente
	
        //Initialize DEFAULT VALUE for timeout of homing procedure, dependent on the range of slit
        //double highSpeedHoming_mm_s;
        //driver->getHighSpeedHoming(highSpeedHoming_mm_s); // highSpeedHoming_mm_s < 0
        //DPRINT("highSpeedHoming_mm_s nell'init=%f",highSpeedHoming_mm_s);
        
        //range_mm = RANGE_MM_DEFAULT;
        //DPRINT("range_mm=%f",range_mm);
        
        //timeo_homing_ms = (uint64_t)((range_mm/(-highSpeedHoming_mm_s/2))*1000);
        //DPRINT("Valore calcolato per l'homing: %lu",timeo_homing_ms);
        
        //homing procedure parameters
        internalHomingState=0;
        //state0activated = false;
        
        // now the motor is ready!
        //eventOnMotionCompleteSet = false;
        readyState = true;
	return 0;
    }
   ERR("error parsing initialization string:\"%s\" .Technosoft and channel object was not initialized.",params.c_str());
   
   return -6;
}

int ActuatorTechnoSoft::deinit(){
    readyState=false;
    if (driver!=NULL) {
        delete driver;
    }
    DPRINT("ActuatorTechnoSoft object is deinitialized");
    return 0; 
}

int ActuatorTechnoSoft::moveRelativeMillimeters(double deltaMillimeters){
    DPRINT("moving relative %f mm",deltaMillimeters);
    
    // Calcolo argomento funzione moveRelativeSteps
    double deltaMicroSteps = round((STEPS_PER_ROUNDS_DEFAULT*N_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX){ // solo per adesso e necessario questo filtro..
        return -1;
    }    
    
    if(driver->selectAxis()<0){
        return -2;
    } 
    //long deltaMicroStepsL = deltaMicroSteps;
    if(driver->moveRelativeSteps((long)deltaMicroSteps)<0){
        return -3;
    }
    return 0;
}

//int ActuatorTechnoSoft::moveRelativeMillimetersHoming(double deltaMillimeters){
//    
//    DPRINT("moving relative %f mm",deltaMillimeters);
//    
//    // Calcolo argomento funzione moveRelativeSteps
//    double deltaMicroSteps = round((STEPS_PER_ROUNDS_DEFAULT*N_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
//    
//    if(deltaMicroSteps<LONG_MIN || deltaMicroSteps>LONG_MAX){ 
//        printf("Out of range\n");
//        return -1;
//    }
//    if(driver->selectAxis()<0){
//        return -2;
//    }
//
//    //long deltaMicroStepsL = deltaMicroSteps;
//    if(driver->moveRelativeStepsHoming((long)deltaMicroSteps)<0){
//        return -3;
//    }
//    
//    return 0;
//}

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

// Set homing parameters
int ActuatorTechnoSoft::sethighSpeedHoming(double speed){
    if(driver->sethighSpeedHoming(speed)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setlowSpeedHoming(double speed){
    if(driver->setlowSpeedHoming(speed)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setAccelerationHoming(double acceleration){
    if(driver-> setaccelerationHoming(acceleration)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setAdditiveHoming(bool isAdditive){
    if(driver->setAdditiveHoming((int)isAdditive)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setMovementHoming(int32_t movement){
    if(driver->setMovementHoming((short)movement)<0){
        return -1;
    }
    return 0;
}

int ActuatorTechnoSoft::setReferenceBaseHoming(int32_t referenceBase){
    if(driver->setReferenceBaseHoming((short)referenceBase)<0){
        return -1;
    }                
    return 0;
}

// Move absolute homing
int ActuatorTechnoSoft::moveAbsoluteMillimeters(double millimeters){ 
    
    // Calcolo argomento funzione moveAbsoluteSteps
    double nMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    printf("nMicroSteps=%f\n",nMicroSteps);
    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX){ // solo per adesso e necessario questo filtro..
        return -1;
    }
    if(driver->selectAxis()<0){
        return -2;
    }

    //long nMicroStepsL = nMicroSteps;
    if(driver->moveAbsoluteSteps((long)nMicroSteps)<0){     
        return -3;
    }
    return 0;
}

//int ActuatorTechnoSoft::moveAbsoluteMillimetersHoming(double millimeters){ 
//    
//
//    // Calcolo argomento funzione moveAbsoluteSteps
//    double nMicroSteps = round((STEPS_PER_ROUNDS_DEFAULT*N_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
//    printf("nMicroSteps=%f\n",nMicroSteps);
//    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX){ // solo per adesso e necessario questo filtro..
//        return -1;
//    }
//    
//    if(driver->selectAxis()<0){
//        return -2;
//    }
//    
//    //long nMicroStepsL = nMicroSteps;
//    if(driver->moveAbsoluteStepsHoming((long)nMicroSteps)<0){    
//        return -3;
//    }
//    return 0;
//}

int ActuatorTechnoSoft::getPosition(readingTypes mode, double* deltaPosition_mm){
     
    DPRINT("Position reading");
    if(driver->selectAxis()<0){
        return -1;
    }

    if(mode==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        long tposition;
        if(driver->getCounter(tposition)<0){
            DERR("getting counter");
            return -2;
        }
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        *deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*N_ROUNDS_DEFAULT);
    }
    else if(mode==READ_ENCODER){ // Lettura posizione per mezzo dell'encoder (Apos register)
        long aposition;
        
        if(driver->getEncoder(aposition)<0)
            return -3;
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        *deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(N_ENCODER_LINES_DEFAULT*N_ROUNDS_DEFAULT);
    }
    return 0;
}

int ActuatorTechnoSoft::homing(homingType mode){
    // Attenzione: la variabile mode non viene utilizzata
    int risp;
    int switchTransited=0;
    int motionCompleted = 0;
    std::string cappos = "CAPPOS";
    long cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
    int absoluteMotionCompleted = 0;
    
    switch (internalHomingState) {
        case 0:
            if(driver->selectAxis()<0){
                internalHomingState = 0;
                risp = -1;
            }
            if(driver->moveVelocityHoming()<0){
                internalHomingState = 0;
                if(driver->stopMotion()<0){
                    risp = -2;
                }
                risp = -3;
            }
            DPRINT(" STATE 0: move velocity activated ");
            if(driver->setEventOnLimitSwitch()<0){
                internalHomingState = 0;
                if(driver->stopMotion()<0){
                    risp = -4;
                }
                risp = -5;
            }
            DPRINT("STATE 0: event on limit switch activated ");
            internalHomingState = 1;
            risp = 1;
            break; 
        case 1:
            if(driver->selectAxis()<0){
                internalHomingState = 0;
                risp = -6;
            }
            if(driver->checkEvent(switchTransited)<0){
                internalHomingState = 0;// deve essere riinizializzato per successivi nuovi tentativi di homing 
                if(driver->stopMotion()<0){
                    risp = -7;
                }    
                risp = -8;
            } 
            DPRINT(" STATE 1: possible limit switch transition just checked ");
            if(switchTransited){
                if(driver->setEventOnMotionComplete()<0){ 
                    internalHomingState = 0; // deve essere riinizializzato per successive operazione di homing
                    if(driver->stopMotion()<0){
                        risp= -9;
                    }
                    risp =-10;
                }  
                //eventOnMotionCompleteSet = true;
                internalHomingState=2;
                DPRINT(" STATE 1: Negative limit switch transited. Event on motion completed set ");
            }
            // **************DA IMPLEMENTARE:*****************
            // RESET ON Limit Switch Transition Event
            risp = 1;
            break; 
        case 2:
            if(driver->selectAxis()<0){
                internalHomingState = 0;
                risp = -11;
            }
            if(driver->checkEvent(motionCompleted)<0){
                internalHomingState = 0;
                if(driver->stopMotion()<0){
                    risp = -12;
                }
                risp= -13;
            }
            DPRINT("************** STATE 2: possible event on motion completed checked **************");
            if(motionCompleted){
                DPRINT("************** STATE 2: Motion completed after transition **************");
                internalHomingState = 3;
            }
            risp= 1;
            break;
        case 3:
            // The motor is not in motion
            DPRINT("************** STATE 3: read the captured position on limit switch transition**************");
            if(driver->selectAxis()<0){
                internalHomingState = 0;
                risp = -13;
            }
            if(driver->getLVariable(cappos, cap_position)<0){ 
    //                if(driver->stopMotion()<0){
    //                    return -13;
    //                }
                internalHomingState=0;
                risp = -14;
            }
            DPRINT("************** STATE 3: the captured position on limit switch transition is %ld [drive internal position units]**************",cap_position);
        
            /*	Command an absolute positioning on the captured position */
            if(driver->moveAbsoluteStepsHoming(cap_position)<0){  
                internalHomingState=0;
                risp = -15;
            }
            DPRINT("************** STATE 3: command of absolute positioning on the captured position sended **************");
            internalHomingState = 4;
            risp= 1;
            break;
        case 4:
            DPRINT("************** STATE 4: wait for positioning to end **************");
            if(driver->selectAxis()<0){
                internalHomingState = 0;
                risp = -16;
            }
//        if(!eventOnMotionCompleteSet){
//            if(driver->setEventOnMotionComplete()<0){
//                if(driver->stopMotion()<0){
//                    eventOnMotionCompleteSet = false;
//                    internalHomingState = 0;
//                    return -12;
//                }
//                eventOnMotionCompleteSet = false;
//                internalHomingState = 0;
//                return -13;
//            } 
//        }
            if(driver->checkEvent(absoluteMotionCompleted)<0){
                internalHomingState = 0;
                if(driver->stopMotion()<0){
                    //eventOnMotionCompleteSet = false;
                    risp= -17;
                }
                //eventOnMotionCompleteSet = false;
                risp= -18;
            }
            if(absoluteMotionCompleted){
                internalHomingState = 5;
                DPRINT("************** STATE 4: motor positioned to end **************");
            }
            // **************DA IMPLEMENTARE:*****************
            // RESET ON Event On Motion Complete
            risp= 1;
            break;
        case 5:
            // The motor is positioned to end
            if(driver->selectAxis()<0){
                internalHomingState = 0;
                risp = -19;
            }
        
            if(driver->resetEncoder()<0){
                internalHomingState = 0;
                //if(driver->stopMotion()<0){
                //return -23;
                //}
                risp= -20;
            }
            if(driver->resetCounter()<0){
                internalHomingState = 0;
                //if(driver->stopMotion()<0){
                    //return -25;
                //}
                risp= -21;
            }
            DPRINT("************** STATE 5: encoder e counter e counter are reset **************");
            internalHomingState = 0;
            risp= 0;
            break;
        default:
            risp= -22;
            break; 
    } 
    return risp;
}


int ActuatorTechnoSoft::getState(int* state, std::string& descStr){

    DPRINT("Getting state of the actuator. ");

    *state  = ACTUATOR_UNKNOWN_STATUS;
    descStr.assign("");

    int stCode=0;

    uint16_t contentRegSRH; // remember typedef uint16_t WORD;
    uint16_t contentRegSRL;
    
    if(driver->selectAxis()<0){
        return -1;
    }

    short indexReg = 4; // see constant REG_SRH in TML_lib.h
    if((driver->getStatusOrErrorReg(indexReg, contentRegSRH, descStr))<0){
        ERR("Reading state error: %s",descStr.c_str());
        return -2;
    }
    indexReg = 3; // see constant REG_SRL in TML_lib.h
    if((driver->getStatusOrErrorReg(indexReg, contentRegSRL, descStr))<0){
        ERR("Reading state error: %s",descStr.c_str());
        return -3;
    }

    if(readyState){ // readyState = true se la procedura di inizializzazione è andata a buon fine. Accendo il primo bit
        stCode|=ACTUATOR_READY;
        descStr=descStr+"Ready. ";
    }

//    // Analysis of the register content SRH (bit 1,2,3,4)
//    bool overPositionTrigger = false;
//    for(int i=1; i<=4; i++){
//        if(contentRegSRH & ((uint16_t)1 << i)){
//            stCode |= ACTUATOR_OVER_POSITION_TRIGGER; // accendo il secondo bit
//            overPositionTrigger = true;
//        }
//    }

//    if(overPositionTrigger){
//        //desc.assign("Position trigger. "); // **************DA QUI IN POI LA STRINGA DOVRÀ ESSERE CONCATENATA
//        descStr=descStr+"Over position trigger. ";
//    }
    // con il contenuto corrente **************
    if(contentRegSRH & ((uint16_t)1<<5)){
        stCode |= ACTUATOR_AUTORUN_ENABLED;
        descStr+="Auto run mode. ";
    }
    if(contentRegSRH & ((uint16_t)1<<6)){
        stCode |= ACTUATOR_LSP_EVENT_INTERRUPUT;
        descStr+="Limit switch positive event/interrupt. ";
    }
//    if(contentRegSRH & ((uint16_t)1<<7)){
//        stCode |= ACTUATOR_LSN_EVENT_INTERRUPT;
//        descStr+="Limit switch negative event/interrupt. ";
//    }
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
//    //  Analysis of the register content SRL
//    if(contentRegSRL & ((uint16_t)1<<10)){
//        stCode |= ACTUATOR_MOTION_COMPLETED;
//        descStr+="Actuator motion completed. ";
//    }
    if(contentRegSRL & ((uint16_t)1<<15)){
        stCode |= ACTUATOR_POWER_SUPPLIED;
        descStr += "Electrical power supplied.";
    }
     
    // Homing in progress state
    if(internalHomingState>0){
        stCode |= HOMING_IN_PROGRESS;
        descStr += "Homing in progress.";
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
    
    if(driver->selectAxis()<0){
        return -1;
    }
    
    short indexRegMER = 5; // see constant REG_MER in TML_lib.h
    if(driver->getStatusOrErrorReg(indexRegMER, contentRegMER, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        return -2;
    }

    short indexRegSRH = 4; // see constant REG_SRH in TML_lib.h
    if(driver->getStatusOrErrorReg(indexRegSRH, contentRegSRH, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        return -3;
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
                descStr=descStr+"CAN bus error. ";
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
                descStr= descStr+ "Communication error. ";
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
        descStr+="Drive I2T protection warning";
    }

    *alrm = stCode;
    return 0;
}
     
 int ActuatorTechnoSoft::stopMotion(){
     if(driver->selectAxis()<0){
        return -1;
     }
     if(driver->stopMotion()<0){
        return -2;
     }
     return 0;
 }

 int ActuatorTechnoSoft::resetAlarms(uint64_t mode){
     
     // In the fault status the power stage is disabled, the MER register signals
     // the errors occurred and  bit 15 from the SRH is set to high to signal the fault state
     int err = 0;
     if(driver->selectAxis()<0){
        return -1;
     }
     switch(mode){
         case 0:
            if(driver->resetFault()<0){
                err = -2;
                // Note: the drive-motor will return to FAULT status (SRH.15=1) if there are
                // errors when the function is executed)
            }
            DPRINT("FUNZIONE DI RESET ALARMS ESEGUITA");
            break;
         case 1:
//            if(driver->resetSetup()<0){
//                err = -2;
//                // Note: the drive-motor will return to FAULT status (SRH.15=1) if there are
//                // errors when the function is executed)
//            }
//            DPRINT("FUNZIONE DI RESET STATE ESEGUITA");
//            break;
             err = -3;
             break;
         default:
             err = -4;
             break;
     }
     return err;
 }
 
 int ActuatorTechnoSoft::getSWVersion(std::string& version){
     if(driver->selectAxis()<0){
        return -1;
     }
     char firmVers[100];
     if(driver->getFirmwareVers(&firmVers[0])<0){
        version = "No firmware version retrivied";
        return -2;
     }
     version.assign(firmVers);
     return 0;
 }
 
int ActuatorTechnoSoft::poweron(uint32_t timeo_ms){
     
    if(driver->selectAxis()<0){
        return -1;
    }
    
    if(driver->providePower()<0){
        return -2;
    }
    return 0;
 }
