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
//#include <boost/algorithm/string/trim.hpp>
//#include <boost/algorithm/string/case_conv.hpp>

#include <boost/algorithm/string.hpp>  
#include <boost/algorithm/string/trim.hpp>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>         // std::locale, std::toupper

using namespace boost;
using namespace ::common::actuators::models;
        
//([\\w\\/]+)int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
      

static const boost::regex driver_match1("(\\d+),(\\d+),(\\d+),(.+)");
// initialisation format <device>,<device name>,<configuration path>,<axisid>,<hostid> (\\w+)
static const boost::regex driver_match2("(\\d+),(.+)");

ActuatorTechnoSoft::ActuatorTechnoSoft(){
    driver=NULL;
    readyState=false;
    partialInit = false;
}

ActuatorTechnoSoft::~ActuatorTechnoSoft(){
    deinit();
    DPRINT("Deallocazione oggetto actuator TechnSoft");
}

// La nuova funzione init si dovra' occupare della sola inizializzazione del canale
// quindi la stringa dovra' contenere informazioni necessarie per la sola eventuale 
// apertura del canale
int ActuatorTechnoSoft::init(void*initialization_string){
    
    if(readyState || partialInit){ // Nel caso venga richiamata la procedura di inizializzazione di nuovo sullo stesso oggetto gia' inizializzato
        DPRINT("already initialized channel and motor");
        return -1;
    }
    
    std::string params;
    params.assign((const char*)initialization_string);
    boost::smatch match;
    
    DPRINT("Initialization string %s", params.c_str());
    
    if(regex_match(params, match, driver_match1, boost::match_extra)){
        
        std::string strHostID = match[1]; 
        hostID = atoi(strHostID.c_str());
        //printf("hostID = %d",hostID);
        //dev_name=match[2]; // nome seriale
        std::string strbtType = match[2];
        btType = atoi(strbtType.c_str());
        //printf("btType = %d",btType);
        std::string strbaudrate = match[3]; 
        baudrate = atoi(strbaudrate.c_str());
        //printf("baudrate = %d",baudrate);
        dev_name=match[4]; // nome seriale
        
        DPRINT("String is matched: hostID: %d, btType: %d, baudrate: %d,serial channel %s",hostID ,btType ,baudrate,dev_name.c_str());
        
        //ATTENZIONE: DEVE ESSERE GESTITE LE SEGUENTI ESPRESSIONI REGOLARI.
        // IL CONTROLLO DEI VALORI APPARTENENTI AL RANGE DI AMMISSIBILITA' DOVRA' ESSERE EFFETTUATO A QUESTO PUNTO
        try{
            driver = new TechnoSoftLowDriver(hostID,dev_name,btType,baudrate);
            // ATTENZIONE: IL COSTRUTTORE TechnoSoftLowDriver DOVRÀ IN SEGUITO 
            // GENERARE UNA ECCEZIONE nel caso l'oggetto SerialCommChannelTechnosoft 
            // non venga allocato.
            // L'istruzione precedente dovrà essere invocata facendo uso di un blocco 
            // try/catch. Nel caso l'eccezione venga catturata dovrà essere restituito
            // un numero negativo
            partialInit = true;
            return 0;
        }
        catch(std::bad_alloc& e){ // Eccezione derivante dal fatto che l'oggetto TechnoSoftLowDriver oppure
                                  // l'oggetto canale di comunicazione non è stato possibile allocarlo.
            ERR("## cannot create TechnoSoftLowDriver or channel object");
            if(driver!=NULL){ 
                delete driver;
                driver = NULL;
            }
            //readyState = false; //non è necessari questa assegnazione
            return -2;
        }
        catch(OpeningChannelException e){
        
            e.badOpeningChannelInfo();
            if(driver!=NULL){ 
                delete driver;
                driver = NULL;
            }
        return -3;
        }
    }
    ERR("error parsing initialization string:\"%s\" .Technosoft and channel object was not initialized.",params.c_str());
    return -4;
}


int ActuatorTechnoSoft::configAxis(void*initialization_string){
    std::string params;
    if(readyState==true){
        DPRINT("already initialized channel and motor");
        return -1;
    }
    params.assign((const char*)initialization_string);
    boost::smatch match;

    DPRINT("Configuration string %s", params.c_str());
    
    if(regex_match(params, match, driver_match2, boost::match_extra)){
//        dev=match[1];      
//        dev_name=match[2];
        std::string conf_path=match[2];
        std::string straxid=match[1];
        int axid = atoi(straxid.c_str());
//        std::string strHostID=match[5];
//        int hostID = atoi(strHostID.c_str());
        
        DPRINT("Configuration axis \"%d\" from path:\"%s\"",axid,conf_path.c_str());
        
        int val;
        if((val=driver->init(conf_path,axid))<0){
            ERR("****************Iipologia di errore in fase di inizializzazione dell'oggetto technsoft low driver %d",val);
            // Deallocazione oggetto canale ()
            //try{
            if(driver!=NULL){
                delete driver;
            }          // Nota importante: l'indirizzo del canale 
                           // non è stato ancora memorizzato nella mappa. Essendo l'oggetto
                           // canale puntato da un solo oggetto TechnoSOftLowDriver,
                           // anche l'oggetto canale sarà deallocato (funzionamento smartpointer).
                           // Se il canale di comunicazione è stato aperto (abbiamo quindi avuto
                           // un errore nella inizializzazione del drive/motor), il canale di 
                           // comunicazione verrà anche chiuso nella fase di deallocazione dell'oggetto
                           // SerialCommChannel
            // readyState = false; // non è necessaria questa istruzione
            driver = NULL;
                
            //}
//            catch(ElectricPowerException e){ //DOVRA' CATTURARE UN'ECCEZIONE DEFINITA DALL'UTENTE CHE
//                     // STA AD INDICARE IL MANCATO SPEGNIMENTO DELL'ALIMENTAZIONE
//                     // DEL MOTORE.
//                     //.....
//                     //.....
//                     //.....
//                e.badElectricPowerInfo();
//                return -3;
//            }
//            catch(StopMotionException e){ //DOVRA' CATTURARE UN'ECCEZIONE DEFINITA DALL'UTENTE CHE
//                     // STA AD INDICARE IL MANCATO SPEGNIMENTO DELL'ALIMENTAZIONE
//                     // DEL MOTORE.
//                     //.....
//                     //.....
//                     //.....
//                e.badStopMotionInfo();
//                return -4;
//            }
            
            return -2;
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
        internalHomingStateDefault=0;
        internalHomingStateHoming2=0;
        //state0activated = false;
        
        // now the motor is ready!
        //eventOnMotionCompleteSet = false;
        readyState = true;
	return 0;
    }
   ERR("error parsing initialization string:\"%s\" ",params.c_str());
   
   return -3;
}

ActuatorTechnoSoft::ActuatorTechnoSoft(const ActuatorTechnoSoft& objActuator){ // OVERLOADING COSTRUTTORE DI COPIA
    
    //std::string dev; // Serial channel name
    //dev = objActuator.dev;
    //std::string dev_name; // ActuatorTechnoSoft name
    dev_name = objActuator.dev_name;
    
    readyState = objActuator.readyState;
    internalHomingStateDefault = objActuator.internalHomingStateDefault;
    internalHomingStateHoming2 = objActuator.internalHomingStateHoming2;
    
    // GESTIONE OGGETTO TechnoSoftLowDriver
    driver = new TechnoSoftLowDriver(hostID,dev_name,btType,baudrate);
    // Inizializzazione membri dell' oggetto TechnoSoftLowDriver
    driver->axisID = (objActuator.driver)->axisID;
    driver->axisRef = (objActuator.driver)->axisRef;
    driver->n_encoder_lines = (objActuator.driver)->n_encoder_lines;
    driver->const_mult_technsoft = (objActuator.driver)->const_mult_technsoft;
    driver->steps_per_rounds = (objActuator.driver)->steps_per_rounds;
    driver->n_rounds = (objActuator.driver)->n_rounds;
    driver->linear_movement_per_n_rounds = (objActuator.driver)->linear_movement_per_n_rounds;
    driver->speed_mm_s = (objActuator.driver)->speed_mm_s;
    driver->maxSpeed_mm_s = (objActuator.driver)->maxSpeed_mm_s;
    driver->acceleration_mm_s2 = (objActuator.driver)->acceleration_mm_s2;
    driver->maxAcceleration_mm_s2 = (objActuator.driver)->maxAcceleration_mm_s2;
    driver->isAdditive = (objActuator.driver)->isAdditive;
    driver->movement = (objActuator.driver)->movement;
    driver->referenceBase = (objActuator.driver)->referenceBase;
    
    driver->highSpeedHoming_mm_s = (objActuator.driver)->highSpeedHoming_mm_s;
    driver->lowSpeedHoming_mm_s = (objActuator.driver)->lowSpeedHoming_mm_s;
    driver->maxHighSpeedHoming_mm_s = (objActuator.driver)->maxHighSpeedHoming_mm_s;
    driver->maxLowSpeedHoming_mm_s = (objActuator.driver)->maxLowSpeedHoming_mm_s;
    driver->accelerationHoming_mm_s2 = (objActuator.driver)->accelerationHoming_mm_s2;
    driver->maxAccelerationHoming_mm_s2 = (objActuator.driver)->maxAccelerationHoming_mm_s2;
    driver->isAdditiveHoming = (objActuator.driver)->isAdditiveHoming;
    driver->movementHoming = (objActuator.driver)->movementHoming;
    driver->referenceBaseHoming = (objActuator.driver)->referenceBaseHoming;
    
    driver->my_channel = (objActuator.driver)->my_channel;
    //driver->channels = (objActuator.driver)->channels;
    //driver->alreadyopenedChannel = (objActuator.driver)->alreadyopenedChannel;
    driver->poweron = (objActuator.driver)->poweron;
    //driver->channelJustOpened = (objActuator.driver)->channelJustOpened;
    
    DPRINT("Costruttore di copia eseguito");
}

ActuatorTechnoSoft& ActuatorTechnoSoft::operator=(const ActuatorTechnoSoft& objActuator){ // Overloading operatori di assegnamento
    
    if(this==& objActuator){
        return *this;
    } 
    //std::string dev; // Serial channel name
    //dev = objActuator.dev;
    //std::string dev_name; // ActuatorTechnoSoft name
    dev_name = objActuator.dev_name;
    
    readyState = objActuator.readyState;
    internalHomingStateDefault = objActuator.internalHomingStateDefault;
    internalHomingStateHoming2 = objActuator.internalHomingStateHoming2;
    
    if(driver!=NULL){ //Prima distruggiamo il vecchio oggetto tec   technosoftLowdriver, per evitare un 
                      // un errore di memori leak. Infatti la copia viene eseguita su un oggetto gia' ESISTENTE!
        delete driver;
        driver = new TechnoSoftLowDriver(hostID,dev_name,btType,baudrate);
    }
    
    // copia valori membri oggetto TechnoSoftLowDriver a destra dell'uguale
    driver->axisID = (objActuator.driver)->axisID;
    driver->axisRef = (objActuator.driver)->axisRef;
    driver->n_encoder_lines = (objActuator.driver)->n_encoder_lines;
    driver->const_mult_technsoft = (objActuator.driver)->const_mult_technsoft;
    driver->steps_per_rounds = (objActuator.driver)->steps_per_rounds;
    driver->n_rounds = (objActuator.driver)->n_rounds;
    driver->linear_movement_per_n_rounds = (objActuator.driver)->linear_movement_per_n_rounds;
    driver->speed_mm_s = (objActuator.driver)->speed_mm_s;
    driver->maxSpeed_mm_s = (objActuator.driver)->maxSpeed_mm_s;
    driver->acceleration_mm_s2 = (objActuator.driver)->acceleration_mm_s2;
    driver->maxAcceleration_mm_s2 = (objActuator.driver)->maxAcceleration_mm_s2;
    driver->isAdditive = (objActuator.driver)->isAdditive;
    driver->movement = (objActuator.driver)->movement;
    driver->referenceBase = (objActuator.driver)->referenceBase;
    
    driver->highSpeedHoming_mm_s = (objActuator.driver)->highSpeedHoming_mm_s;
    driver->lowSpeedHoming_mm_s = (objActuator.driver)->lowSpeedHoming_mm_s;
    driver->maxHighSpeedHoming_mm_s = (objActuator.driver)->maxHighSpeedHoming_mm_s;
    driver->maxLowSpeedHoming_mm_s = (objActuator.driver)->maxLowSpeedHoming_mm_s;
    driver->accelerationHoming_mm_s2 = (objActuator.driver)->accelerationHoming_mm_s2;
    driver->maxAccelerationHoming_mm_s2 = (objActuator.driver)->maxAccelerationHoming_mm_s2;
    driver->isAdditiveHoming = (objActuator.driver)->isAdditiveHoming;
    driver->movementHoming = (objActuator.driver)->movementHoming;
    driver->referenceBaseHoming = (objActuator.driver)->referenceBaseHoming;
    
    driver->my_channel = (objActuator.driver)->my_channel;
    //driver->channels = (objActuator.driver)->channels;
    //driver->alreadyopenedChannel = (objActuator.driver)->alreadyopenedChannel;
    driver->poweron = (objActuator.driver)->poweron;
    //driver->channelJustOpened = (objActuator.driver)->channelJustOpened;
    
    DPRINT("Operatore di assegnamento eseguito");
    return *this;
}

int ActuatorTechnoSoft::deinit(){
    readyState=false;
    if (driver!=NULL) {
        delete driver;
    }
    DPRINT("ActuatorTechnoSoft object is deinitialized");
    return 0; 
}

void trim2(std::string& str){
  std::string::size_type pos = str.find_last_not_of(' ');
  if(pos != std::string::npos) {
    str.erase(pos + 1);
    pos = str.find_first_not_of(' ');
    if(pos != std::string::npos){ 
        str.erase(0, pos);
        }
  }
  else str.erase(str.begin(), str.end());
}

void setUpperCase(const std::string& str,std::string& strResult){
    std::locale loc;
    strResult.assign("");
    for (std::string::size_type i=0; i<str.length(); ++i){
        strResult += std::toupper(str[i],loc);
    }     
}

bool to_bool(const std::string & s) {
     return s != "0";
}

int ActuatorTechnoSoft::setParameter(std::string parName,std::string valueOfparName){
    
    // trim
    trim2(parName);
    trim2(valueOfparName);
    // To upper case
    std::string strResultparName;
    std::string strResultparvalue;
    setUpperCase(parName,strResultparName);
    setUpperCase(valueOfparName,strResultparvalue);
    
    double doubleValue;
    int intValue;
    bool boolValue;
    
//    if(strResultparName.compare("MAXSPEED")==0){
//        doubleValue = atof(valueOfparName.c_str());
//        if(driver->setMaxSpeed(doubleValue)<0){ 
//            return -1;
//        }
//        return 0;
//    } 
    if(strResultparName.compare("SPEED")==0){ 
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setSpeed(doubleValue)<0){
            return -2;
        }
        return 0;
    }   
//    else if(strResultparName.compare("MAXACCELERATION")==0){
//        doubleValue = atof(valueOfparName.c_str());
//        if(driver->setMaxAcceleration(doubleValue)<0){ 
//            return -3;
//        }
//        return 0;
//    } 
    else if(strResultparName.compare("ACCELERATION")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setAcceleration(doubleValue)<0){ 
            return -4;
        }
        return 0;
    }
    else if(strResultparName.compare("ISADDITIVE")==0){
        // Conversion from string to bool
        //boolValue = to_bool(valueOfparName);
        intValue = atoi(valueOfparName.c_str());
        if(driver->setAdditive(intValue)<0){ 
            return -5;
        }
        return 0;
    }
    else if(strResultparName.compare("MOVEMENT")==0){
        intValue = atoi(valueOfparName.c_str());
        if(driver->setMovement((short)intValue)<0){ 
            return -6;
        }
        return 0;
    }
    else if(strResultparName.compare("REFERENCEBASE")==0){
        intValue = atoi(valueOfparName.c_str());
        if(driver->setReferenceBase((short)intValue)<0){ 
            return -7;
        }
        return 0;   
    }
//    else if(strResultparName.compare("MAXHIGHSPEEDHOMING")==0){
//        doubleValue = atof(valueOfparName.c_str());
//        if(driver->setMaxhighSpeedHoming(doubleValue)<0){ 
//            return -8;
//        }
//        return 0;   
//    }
    else if(strResultparName.compare("HIGHSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->sethighSpeedHoming(doubleValue)<0){ 
            return -9;
        }
        return 0;       
    }
//    else if(strResultparName.compare("MAXLOWSPEEDHOMING")==0){
//        doubleValue = atof(valueOfparName.c_str());
//        if(driver->setMaxlowSpeedHoming(doubleValue)<0){ 
//            return -10;
//        }
//        return 0;   
//    }
    else if(strResultparName.compare("LOWSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setlowSpeedHoming(doubleValue)<0){ 
            return -11;
        }
        return 0;   
    }
//    else if(strResultparName.compare("MAXACCELERATIONHOMING")==0){
//        doubleValue = atof(valueOfparName.c_str());
//        if(driver->setMaxAccelerationHoming(doubleValue)<0){ 
//            return -12;
//        }
//        return 0;   
//    }
    else if(strResultparName.compare("ACCELERATIONHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setaccelerationHoming(doubleValue)<0){ 
            return -13;
        }
        return 0;   
    }
    else if(strResultparName.compare("ISADDITIVEHOMING")==0){
        // Conversion from string to bool
        //boolValue = to_bool(valueOfparName);
        intValue = atoi(valueOfparName.c_str());
        if(driver->setAdditiveHoming(intValue)<0){ 
            return -14;
        }
        return 0;
    }
    else if(strResultparName.compare("MOVEMENTHOMING")==0){
        intValue = atoi(valueOfparName.c_str());
        if(driver->setMovementHoming((short)intValue)<0){ 
            return -15;
        }
        return 0;
    }
    else if(strResultparName.compare("REFERENCEBASEHOMING")==0){
        intValue = atoi(valueOfparName.c_str());
        if(driver->setReferenceBaseHoming((short)intValue)<0){ 
            return -16;
        }
        return 0;   
    }//_________________________________________________________________________
    else if(strResultparName.compare("NUMENCODERLINES")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setEncoderLines(doubleValue)<0){ 
            return -17;
        }
        return 0;   
    }
    else if(strResultparName.compare("NUMMICROSTEPSPERSTEP")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setConst_mult_technsoft(doubleValue)<0){ 
            return -18;
        }
        return 0;   
    } 
    else if(strResultparName.compare("STEPSPERROUND")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setSteps_per_rounds(doubleValue)<0){ 
            return -19;
        }
        return 0;   
    } 
    else if(strResultparName.compare("FIXEDNUMBEROFROUNDS")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setN_rounds(doubleValue)<0){ 
            return -20;
        }
        return 0;   
    } 
    else if(strResultparName.compare("LINEARDISPLACEMENT[MM]")==0){
        doubleValue = atof(valueOfparName.c_str());
        if(driver->setLinear_movement_per_n_rounds(doubleValue)<0){ 
            return -21;
        }
        return 0;   
    } 
    else{
        return -22;
    }
}

int ActuatorTechnoSoft::moveRelativeMillimeters(double deltaMillimeters){
    DPRINT("moving relative %f mm",deltaMillimeters);
    
    // Calcolo argomento funzione moveRelativeSteps
    //double deltaMicroSteps = round((STEPS_PER_ROUNDS_DEFAULT*N_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    double deltaMicroSteps=driver->getdeltaMicroSteps(deltaMillimeters);
    DPRINT("deltaMicroSteps = %f mm",deltaMicroSteps);
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

int ActuatorTechnoSoft::setMaxSpeed(double speed){
    if(driver->setMaxSpeed(speed)<0){
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

int ActuatorTechnoSoft::setMaxAcceleration(double maxAcceleration){
    if(driver->setMaxAcceleration(maxAcceleration)<0){
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

int ActuatorTechnoSoft::setMaxhighSpeedHoming(double maxHighSpeed){
    if(driver->setMaxhighSpeedHoming(maxHighSpeed)<0){
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

int ActuatorTechnoSoft::setMaxlowSpeedHoming(double speed){
    if(driver->setMaxlowSpeedHoming(speed)<0){
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

int ActuatorTechnoSoft::setMaxAccelerationHoming(double maxacceleration){
    if(driver-> setMaxAccelerationHoming(maxacceleration)<0){
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

int ActuatorTechnoSoft::setEncoderLines(double _encoderLines){
    if(driver->setEncoderLines(_encoderLines)<0){
        return -1;
    }   
    return 0;
}

int ActuatorTechnoSoft::setConst_mult_technsoft(double _const_mult_technsoft){
    if(driver->setConst_mult_technsoft(_const_mult_technsoft)<0){
        return -1;
    }    
    return 0;
}

int ActuatorTechnoSoft::setSteps_per_rounds(double _steps_per_rounds){
    if(driver->setSteps_per_rounds(_steps_per_rounds)<0){
        return -1;
    }    
    return 0;
}

int ActuatorTechnoSoft::setN_rounds(double _n_rounds){
    if(driver->setN_rounds(_n_rounds)<0){
        return -1;
    }   
    return 0;
}

int ActuatorTechnoSoft::setLinear_movement_per_n_rounds(double _linear_movement_per_n_rounds){
    if(driver->setLinear_movement_per_n_rounds(_linear_movement_per_n_rounds)<0){
        return -1;
    }   
    return 0;
}
           
// Move absolute homing
int ActuatorTechnoSoft::moveAbsoluteMillimeters(double millimeters){ 
    
    // Calcolo argomento funzione moveAbsoluteSteps
    //double nMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    double nMicroSteps = driver->getdeltaMicroSteps(millimeters);
    DPRINT("nMicroSteps=%f\n",nMicroSteps);
    
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
        //long tposition;
        if(driver->getCounter(deltaPosition_mm)<0){
            DERR("getting counter");
            return -2;
        }
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        //*deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*N_ROUNDS_DEFAULT);
    }
    else if(mode==READ_ENCODER){ // Lettura posizione per mezzo dell'encoder (Apos register)
        //long aposition;
        if(driver->getEncoder(deltaPosition_mm)<0){
            return -3;
        }
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        //*deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(N_ENCODER_LINES_DEFAULT*N_ROUNDS_DEFAULT);
    }
    return 0;
}

int ActuatorTechnoSoft::homing(homingType mode){
    // Attenzione: la variabile mode non viene utilizzata
    
    if(mode==defaultHoming){
        
        int risp;
        int switchTransited=0;
        int motionCompleted = 0;
        std::string cappos = "CAPPOS";
        long cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
        int absoluteMotionCompleted = 0;
    
        switch (internalHomingStateDefault) {
            case 0:
                if(driver->selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -1;
                    break;
                }
                if(driver->moveVelocityHoming()<0){
                    internalHomingStateDefault = 0;
                    if(driver->stopMotion()<0){
                        risp = -2;
                        break;
                    }
                    risp = -3;
                    break;
                }
                DPRINT(" STATE 0: move velocity activated ");
                if(driver->setEventOnLimitSwitch()<0){
                    internalHomingStateDefault = 0;
                    if(driver->stopMotion()<0){
                        risp = -4;
                        break;
                    }
                    risp = -5;
                    break;
                }
                DPRINT("STATE 0: event on limit switch activated ");
                internalHomingStateDefault = 1;
                risp = 1;
                break; 
            case 1:
                if(driver->selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -6;
                    break;
                }
                if(driver->checkEvent(switchTransited)<0){
                    internalHomingStateDefault = 0;// deve essere riinizializzato per successivi nuovi tentativi di homing 
                    if(driver->stopMotion()<0){
                        risp = -7;
                        break;
                    }    
                    risp = -8;
                    break;
                } 
                DPRINT(" STATE 1: possible limit switch transition just checked ");
                if(switchTransited){
                    if(driver->setEventOnMotionComplete()<0){ 
                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
                        if(driver->stopMotion()<0){
                            risp= -9;
                            break;
                        }
                        risp =-10;
                        break;
                    }  
                    //eventOnMotionCompleteSet = true;
                    internalHomingStateDefault=2;
                    DPRINT(" STATE 1: Negative limit switch transited. Event on motion completed set ");
                }
                // **************DA IMPLEMENTARE:*****************
                // RESET ON Limit Switch Transition Event
                risp = 1;
                break; 
            case 2:
                if(driver->selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -11;
                    break;
                }
                if(driver->checkEvent(motionCompleted)<0){
                    internalHomingStateDefault = 0;
                    if(driver->stopMotion()<0){
                        risp = -12;
                        break;
                    }
                    risp= -13;
                    break;
                }
                DPRINT("************** STATE 2: possible event on motion completed checked **************");
                if(motionCompleted){
                    DPRINT("************** STATE 2: Motion completed after transition **************");
                    internalHomingStateDefault = 3;
                }
                risp= 1;
                break;
            case 3:
                // The motor is not in motion
                DPRINT("************** STATE 3: read the captured position on limit switch transition**************");
                if(driver->selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -13;
                    break;
                }
                if(driver->getLVariable(cappos, cap_position)<0){ 
    //                if(driver->stopMotion()<0){
    //                    return -13;
    //                }
                    internalHomingStateDefault=0;
                    risp = -14;
                    break;
                }
                DPRINT("************** STATE 3: the captured position on limit switch transition is %ld [drive internal position units]**************",cap_position);
        
            /*	Command an absolute positioning on the captured position */
                if(driver->moveAbsoluteStepsHoming(cap_position)<0){  
                    internalHomingStateDefault=0;
                    risp = -15;
                    break;
                }
                DPRINT("************** STATE 3: command of absolute positioning on the captured position sended **************");
                internalHomingStateDefault = 4;
                risp= 1;
                break;
            case 4:
                DPRINT("************** STATE 4: wait for positioning to end **************");
                if(driver->selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -16;
                    break;
                }
//        if(!eventOnMotionCompleteSet){
//            if(driver->setEventOnMotionComplete()<0){
//                if(driver->stopMotion()<0){
//                    eventOnMotionCompleteSet = false;
//                    internalHomingStateDefault = 0;
//                    return -12;
//                }
//                eventOnMotionCompleteSet = false;
//                internalHomingStateDefault = 0;
//                return -13;
//            } 
//        }
                if(driver->checkEvent(absoluteMotionCompleted)<0){
                    internalHomingStateDefault = 0;
                    if(driver->stopMotion()<0){
                    //eventOnMotionCompleteSet = false;
                        risp= -17;
                        break;
                    }
                //eventOnMotionCompleteSet = false;
                    risp= -18;
                    break;
                }
                if(absoluteMotionCompleted){
                    internalHomingStateDefault = 5;
                    DPRINT("************** STATE 4: motor positioned to end **************");
                }
            // **************DA IMPLEMENTARE:*****************
            // RESET ON Event On Motion Complete
                risp= 1;
                break;
            case 5:
                // The motor is positioned to end
                if(driver->selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -19;
                    break;
                }
        
                if(driver->resetEncoder()<0){
                    internalHomingStateDefault = 0;
                    //if(driver->stopMotion()<0){
                    //return -23;
                    //}
                    risp= -20;
                    break;
                }
                if(driver->resetCounter()<0){
                    internalHomingStateDefault = 0;
                    //if(driver->stopMotion()<0){
                    //return -25;
                    //}
                    risp= -21;
                    break;
                }
                DPRINT("************** STATE 5: encoder e counter e counter are reset **************");
                internalHomingStateDefault = 0;
                risp= 0;
                break;
            default:
                internalHomingStateDefault = 0;
                risp= -22;
                break; 
        } 
        return risp;
    }
    else if(mode==homing2){
        int risp;
        uint16_t contentReg;
        std::string descStr = "";
        short homingDone = 0;
        switch (internalHomingStateHoming2) {
            case 0:
                DPRINT("************** Homing procedure Homing2. STATE 0. **************");
                if(driver->moveVelocityHoming()<0){
                    internalHomingStateHoming2=0;
                    risp= -1;
                    break;
                }
                internalHomingStateHoming2=1;
                risp= 1;
                break;
            case 1:
                DPRINT("************** Homing procedure Homing2. STATE 1. **************");
                if((driver->getStatusOrErrorReg(5, contentReg, descStr))<0){
                    //ERR("Reading state error: %s",descStr.c_str());
                    internalHomingStateHoming2=0;
                    if(driver->stopMotion()<0){
                        risp= -2;
                        break;
                    }
                    risp= -3;
                    break;
                }
                // lettura bit di interesse
                homingDone=((contentReg & ((uint16_t)1)<<7) != 0 ? 1 : 0);
                if(homingDone){
                   internalHomingStateHoming2=2;
                }
                risp= 1;
                break;
            case 2:
                DPRINT("************** Homing procedure Homing2. STATE 2. **************");
                DPRINT("************** Reset encoder e counter **************");
                if(driver->resetEncoder()<0){
                    internalHomingStateHoming2=0;
                    if(driver->stopMotion()<0){
                        risp= -4;
                        break;
                    }
                    risp= -5;
                    break;
                }
                if(driver->resetCounter()<0){
                    internalHomingStateHoming2=0;
                    if(driver->stopMotion()<0){
                        risp= -6;
                        break;
                    }
                    risp= -7;
                    break;
                }
                internalHomingStateHoming2=0;
                risp=0;
                break;
            default:
                internalHomingStateHoming2 = 0;
                risp=-22;
                break;
        }
        return risp;
    }
    else{
        return -100;
    }
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
    //  Analysis of the register content SRL
    if(!(contentRegSRL & ((uint16_t)1<<10))){
        stCode |= ACTUATOR_INMOTION;
        descStr+="Actuator in motion.";
    }
    if(contentRegSRL & ((uint16_t)1<<15)){
        stCode |= ACTUATOR_POWER_SUPPLIED;
        descStr += "Electrical power supplied.";
    }
     
    // Homing in progress state
    if(internalHomingStateDefault>0 || internalHomingStateHoming2>0){
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
     mode =0;
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
 
int ActuatorTechnoSoft::poweron(int on){
    if(driver->selectAxis()<0){
        return -1;
    }
    int resp;
    switch (on) {
        case 0:
            if(driver->stopPower()<0){
                resp=-2;
            } 
            resp=0;
            break;       
        case 1:
            if(driver->providePower()<0){
                resp=-3;
            }
            resp=0;
            break;
        default:
            resp=-4;
            break;
    }
    return resp;
}

int ActuatorTechnoSoft::getHWVersion(std::string& version){
    
   version=" Technosoft IDM stepper open loop mode";
   return 0; 
}
int ActuatorTechnoSoft::sendDataset(std::string& dataset){
   dataset.clear();
   dataset="{\"attributes\":[";
   //dataset+="{\"name\":\"maxSpeed\",\"description\":\"Upper bound for max speed of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"500\"}";
   dataset+="{\"name\":\"speed\",\"description\":\"Max speed of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"500.0\",\"default\":\"400.0\"},";
   //dataset+="{\"name\":\"maxAcceleration\",\"description\":\"Upper bound for acceleration of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\"}"; 
   dataset+="{\"name\":\"acceleration\",\"description\":\"Acceleration of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"2.0\",\"default\":\"0.6\"},";
   dataset+="{\"name\":\"isadditive\",\"description\":\"Specifies how is computed the position to reach\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"0\"},";
   dataset+="{\"name\":\"movement\",\"description\":\"Defines the moment when the motion is started\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"-1\",\"max\":\"1\",\"default\":\"1\"},";
   dataset+="{\"name\":\"referenceBase\",\"description\":\"Specifies how the motion reference is computed\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"1\"},";
   //dataset+="{\"name\":\"maxhighspeedhoming\",\"description\":\"Upper bound for max speed of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\"}";
   dataset+="{\"name\":\"highspeedhoming\",\"description\":\"Max speed of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"15.0\",\"default\":\"10.0\"},";
   //dataset+="{\"name\":\"maxlowspeedhoming\",\"description\":\"Upper bound for speed of trapezoidal profile for homing procedure, for repositioning the slit at LSN switch\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\"}";
   dataset+="{\"name\":\"lowspeedhoming\",\"description\":\"Speed of trapezoidal profile for homing procedure, for repositioning slit at LSN switch\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"3.0\",\"default\":\"1.0\"},";
   //dataset+="{\"name\":\"maxaccelerationhoming\",\"description\":\"Upper bound for acceleration of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\"}";
   dataset+="{\"name\":\"accelerationhoming\",\"description\":\"Acceleration of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"0.6\",\"default\":\"0.3\"},";
   dataset+="{\"name\":\"isadditivehoming\",\"description\":\"Specifies how is computed the position to reach for homing procedure\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"0\"},";
   dataset+="{\"name\":\"movementhoming\",\"description\":\"Defines the moment when the motion is started for homing procedure\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"-1\",\"max\":\"1\",\"default\":\"1\"},";
   dataset+="{\"name\":\"referenceBaseHoming\",\"description\":\"Specifies how the motion reference is computed for homing procedure\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"1\"},";
   
   dataset+="{\"name\":\"numencoderlines\",\"description\":\"Number of encoder lines\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"800\"},";
   dataset+="{\"name\":\"nummicrostepsperstep\",\"description\":\"Number of micro steps per step\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"256\"},";
   dataset+="{\"name\":\"stepsperround\",\"description\":\"Number of steps to perfor a complete round\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"200\"},";
   dataset+="{\"name\":\"fixednumberofrounds\",\"description\":\"Number of rounds for which the linear displacement is known\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"20\"},";
   dataset+="{\"name\":\"lineardisplacement[mm]\",\"description\":\"Linear displacement [mm] performed by slit associated with fixednumberofrounds rounds\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.000000001\",\"max\":\"10000000\",\"default\":\"1.5\"}";
   dataset+="]}";
   
   return 0; 
}
