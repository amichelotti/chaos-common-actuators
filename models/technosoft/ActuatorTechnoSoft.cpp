/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

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
//#include <cstdlib.h>

//#include <boost/algorithm/string/trim.hpp>
//#include <boost/algorithm/string/case_conv.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>         // std::locale, std::toupper

using namespace boost;
using namespace ::common::actuators::models::simul;
using namespace ::common::actuators;

//([\\w\\/]+)int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft


static const boost::regex driver_match1("(\\d+),(\\d+),(\\d+),(.+)");
// initialisation format <device>,<device name>,<configuration path>,<axisid>,<hostid> (\\w+)
static const boost::regex driver_match2("(\\d+),(.+)");

std::map<int,TechnoSoftLowDriver *> ActuatorTechnoSoft::motors;

ActuatorTechnoSoft::ActuatorTechnoSoft(){
    //driver=NULL;
    channel = NULL;
    initChannelAlreadyDone = false;
    //configAxisAlreadyDone = false,
    delectingActuator = false;
}

ActuatorTechnoSoft::~ActuatorTechnoSoft(){
    // show content:
    DPRINT("Deleting Actuator Technosoft");
    delectingActuator = true;
    for (std::map<int,TechnoSoftLowDriver *> ::iterator it=motors.begin(); it!=motors.end(); ++it){
        deinit(it->first);
        //DPRINT("Deallocazione oggetto actuatorTechnSoft con axis ID %d",it->first);
    }
    // Remove all the element from the map container
    motors.clear();

    DPRINT("Verifichiamo ora la dimensione della mappa statica: %d", motors.size());

    // close the communication channel
    if(channel!=NULL){
        delete channel;
        channel = NULL;
    }
    DPRINT("Object Actuator Technosoft is deleted");
}

// La nuova funzione init si dovra' occupare della sola inizializzazione del canale
// quindi la stringa dovra' contenere informazioni necessarie per la sola eventuale
// apertura del canale
int ActuatorTechnoSoft::init(void*initialization_string){

    if(initChannelAlreadyDone){
        DPRINT("This object has already a communication channel correctly initialized");
        return -1;
    }

    std::string params;
    params.assign((const char*)initialization_string);
    boost::smatch match;

    DPRINT("Initialization string %s", params.c_str());

    if(regex_match(params, match, driver_match1, boost::match_extra)){

        std::string strHostID = match[1];
        hostID = atoi(strHostID.c_str());
        std::string strbtType = match[2];
        btType = atoi(strbtType.c_str());
        std::string strbaudrate = match[3];
        baudrate = atoi(strbaudrate.c_str());
        dev_name=match[4];

        DPRINT("String is matched: hostID: %d, btType: %d, baudrate: %d,serial channel %s",hostID ,btType ,baudrate,dev_name.c_str());

        //SerialCommChannelTechnosoft objChannel(hostID, dev_name, btType, baudrate);
        channel = new (std::nothrow) SerialCommChannelTechnosoft(hostID, dev_name, btType, baudrate);
        if(channel==NULL){
            return -2;
        }
        // Tentativo apertura canale di comunicazione, simulato
        if(channel->open()<0){
            return -3;
        }
        std::srand(std::time(0));
        int random_variable = std::rand();
        if(random_variable<(RAND_MAX/20))
            return -3;
        initChannelAlreadyDone = true;
        return 0;
    }
    ERR("Cannot possible init channel %s",params.c_str());
    return -4;
}

int ActuatorTechnoSoft::configAxis(void*initialization_string){
    std::string params;

    params.assign((const char*)initialization_string);
    boost::smatch match;

    DPRINT("Configuration string %s", params.c_str());

    if(regex_match(params, match, driver_match2, boost::match_extra)){

        std::string conf_path=match[2];
        std::string straxid=match[1];
        int axid = atoi(straxid.c_str());

        // Controllo mappa motori
        std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axid); // iteratore alla mappa statica

        if(i==motors.end()){ // Il motore con il corrente axis id non e' stato configurato.
                             // In questo caso creero' un nuovo oggetto motore.
            TechnoSoftLowDriver* driver = new (std::nothrow) TechnoSoftLowDriver();
            if(driver==NULL){
                return -1;
            }
            int val;
            if((val=driver->init(conf_path,axid))<0){
                ERR("****************Iipologia di errore in fase di inizializzazione dell'oggetto technosoft low driver %d",val);
                delete driver;
                driver = NULL;
                return -2;
            }
            DPRINT("Axis id %d configurato correttamente.", axid);
            motors.insert(std::pair<int,TechnoSoftLowDriver*>(axid,driver));
            DPRINT("Dimensione mappa statica alla fine della configurazione dell'axisID %d avvenuta correttamente: %d",axid,motors.size());
            return 0;
        }
        DPRINT("Axis id %d è stato già configurato correttamente.", axid);
        return -3;
        //HOMING procedure parameters
        //**********************************************************************
        //**********************************************************************
        //Le seguenti tre variabili sono relative a ciascun oggetto TechnoSoftLowDriver
        //**********************************************************************
        //**********************************************************************
//        internalHomingStateDefault=0;
//        internalHomingStateHoming2=0;
        // ***********************GESTIONE_READY_STATE*************************
//return 0;
    }
    ERR("error parsing initialization string:\"%s\" ",params.c_str());
    return -4;
}

ActuatorTechnoSoft::ActuatorTechnoSoft(const ActuatorTechnoSoft& objActuator){ // OVERLOADING COSTRUTTORE DI COPIA

    dev_name = objActuator.dev_name;
    initChannelAlreadyDone = objActuator.initChannelAlreadyDone;
    delectingActuator = objActuator.delectingActuator;

    btType = objActuator.btType;
    baudrate = objActuator.baudrate;
    hostID = objActuator.hostID;

    //readyState = objActuator.readyState;
//    internalHomingStateDefault = objActuator.internalHomingStateDefault;
//    internalHomingStateHoming2 = objActuator.internalHomingStateHoming2;

    // GESTIONE OGGETTO SerialCommChannelTechnosoft
    channel = new SerialCommChannelTechnosoft(hostID, dev_name, btType, baudrate);

//    driver = new TechnoSoftLowDriver();
    // Inizializzazione membri dell' oggetto SerialCommChannelTechnosoft
    channel->pszDevName = (objActuator.channel)->pszDevName;
    channel->btType = (objActuator.channel)->btType;
    channel->baudrate = (objActuator.channel)->baudrate;
    channel->hostID = (objActuator.channel)->hostID;
    channel->fd = (objActuator.channel)->fd;

    DPRINT("Costruttore di copia eseguito");
}

ActuatorTechnoSoft& ActuatorTechnoSoft::operator=(const ActuatorTechnoSoft& objActuator){ // Overloading operatori di assegnamento

    if(this==& objActuator){
        return *this;
    }

    dev_name = objActuator.dev_name;
    initChannelAlreadyDone = objActuator.initChannelAlreadyDone;
    delectingActuator = objActuator.delectingActuator;

    btType = objActuator.btType;
    baudrate = objActuator.baudrate;
    hostID = objActuator.hostID;

    if(channel!=NULL){ //Prima distruggiamo il vecchio oggetto tec   technosoftLowdriver, per evitare un
                      // un errore di memori leak. Infatti la copia viene eseguita su un oggetto gia' ESISTENTE!
        delete channel;
        channel = new SerialCommChannelTechnosoft(hostID, dev_name, btType, baudrate);
    }

    // copia valori membri oggetto TechnoSoftLowDriver a destra dell'uguale
    channel->pszDevName = (objActuator.channel)->pszDevName;
    channel->btType = (objActuator.channel)->btType;
    channel->baudrate = (objActuator.channel)->baudrate;
    channel->hostID = (objActuator.channel)->hostID;
    channel->fd = (objActuator.channel)->fd;

    DPRINT("Operatore di assegnamento eseguito");
    return *this;
}

int ActuatorTechnoSoft::deinit(int axisID){
    //readyState=false;

    // Controllo costruzione oggetto axisID
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato, non c'e' quindi alcun motore da inizializzare
        return -1;
    }
    // Invio comando stop di movimentazione al motore
//    if((i->second)->selectAxis()<0)
//        return -2;
    // ************* Simula INVIO comando selectAxis ****************
    int random_variable = std::rand();
    if(random_variable<(RAND_MAX/40))
        return -2;
// ************* Simula INVIO comando stopMotion() ****************
//    if((i->second)->stopMotion()<0)
//        return -3;
    random_variable = std::rand();
    if(random_variable<(RAND_MAX/40))
        return -3;

    if(((i->second)->actuatorIDInMotion)==true)
        // ............... GESTIRE LO STOP VIRTUALE DEL MOTORE ATTRAVERSO IL LOCK ...............

    // Stoppiamo effettivamente il motion e la corrente

//    if((i->second)->stopPower()<0)
//        return -4;
    random_variable = std::rand();
    if(random_variable<(RAND_MAX/40))
        return -4;

    // Invio comando apertura circuito alimentazione motore
    if(i->second!=NULL){
        delete (i->second);
        i->second = NULL;
    }

    if(!delectingActuator){
        motors.erase(axisID);
        if(motors.size()==0){
            if(channel!=NULL){
                delete channel;
                channel = NULL;
            }
        }
    }

    DPRINT("Object technosoftlowdriver with axisID = %d is deinitialized",axisID);
    // Controllo lista vuota. Se e' vuota bisogna chiudere il canale!!!!!
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

int ActuatorTechnoSoft::setParameter(int axisID,std::string parName,std::string valueOfparName){

    //git addaaaaaaaaaaaaaa

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

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

    if(strResultparName.compare("SPEED")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setSpeed(doubleValue)<0){
            return -2;
        }
        return 0;
    }
    else if(strResultparName.compare("ACCELERATION")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setAcceleration(doubleValue)<0){
            return -3;
        }
        return 0;
    }
    else if(strResultparName.compare("ISADDITIVE")==0){
        // Conversion from string to bool
        //boolValue = to_bool(valueOfparName);
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setAdditive(intValue)<0){
            return -4;
        }
        return 0;
    }
    else if(strResultparName.compare("MOVEMENT")==0){
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setMovement((short)intValue)<0){
            return -5;
        }
        return 0;
    }
    else if(strResultparName.compare("REFERENCEBASE")==0){
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setReferenceBase((short)intValue)<0){
            return -6;
        }
        return 0;
    }
    else if(strResultparName.compare("HIGHSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->sethighSpeedHoming(doubleValue)<0){
            return -7;
        }
        return 0;
    }

    else if(strResultparName.compare("LOWSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setlowSpeedHoming(doubleValue)<0){
            return -8;
        }
        return 0;
    }
    else if(strResultparName.compare("ACCELERATIONHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setaccelerationHoming(doubleValue)<0){
            return -9;
        }
        return 0;
    }
    else if(strResultparName.compare("ISADDITIVEHOMING")==0){
        // Conversion from string to bool
        //boolValue = to_bool(valueOfparName);
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setAdditiveHoming(intValue)<0){
            return -10;
        }
        return 0;
    }
    else if(strResultparName.compare("MOVEMENTHOMING")==0){
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setMovementHoming((short)intValue)<0){
            return -11;
        }
        return 0;
    }
    else if(strResultparName.compare("REFERENCEBASEHOMING")==0){
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setReferenceBaseHoming((short)intValue)<0){
            return -12;
        }
        return 0;
    }//_________________________________________________________________________
    else if(strResultparName.compare("NUMENCODERLINES")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setEncoderLines(doubleValue)<0){
            return -13;
        }
        return 0;
    }
    else if(strResultparName.compare("NUMMICROSTEPSPERSTEP")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setConst_mult_technsoft(doubleValue)<0){
            return -14;
        }
        return 0;
    }
    else if(strResultparName.compare("STEPSPERROUND")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setSteps_per_rounds(doubleValue)<0){
            return -15;
        }
        return 0;
    }
    else if(strResultparName.compare("FIXEDNUMBEROFROUNDS")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setN_rounds(doubleValue)<0){
            return -16;
        }
        return 0;
    }
    else if(strResultparName.compare("LINEARDISPLACEMENT[MM]")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setLinear_movement_per_n_rounds(doubleValue)<0){
            return -17;
        }
        return 0;
    }
    else{
        return -18;
    }
}

int ActuatorTechnoSoft::moveRelativeMillimeters(int axisID,double deltaMillimeters){
    DPRINT("moving relative %f mm",deltaMillimeters);

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

    // Calcolo argomento funzione moveRelativeSteps
    //double deltaMicroSteps = round((STEPS_PER_ROUNDS_DEFAULT*N_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*deltaMillimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    double deltaMicroSteps=(i->second)->getdeltaMicroSteps(deltaMillimeters);
    DPRINT("deltaMicroSteps = %f mm",deltaMicroSteps);
    if(deltaMicroSteps<=LONG_MIN || deltaMicroSteps>=LONG_MAX){ // solo per adesso e necessario questo filtro..
        return -2;
    }

    if((i->second)->selectAxis()<0){
        return -3;
    }

    //long deltaMicroStepsL = deltaMicroSteps;
    if((i->second)->moveRelativeSteps((long)deltaMicroSteps)<0){
        return -4;
    }
    return 0;
}

// Move absolute homing
int ActuatorTechnoSoft::moveAbsoluteMillimeters(int axisID,double millimeters){

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

    // Calcolo argomento funzione moveAbsoluteSteps
    //double nMicroSteps = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*millimeters)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    double nMicroSteps = (i->second)->getdeltaMicroSteps(millimeters);
    DPRINT("nMicroSteps=%f\n",nMicroSteps);

    if(nMicroSteps<=LONG_MIN || nMicroSteps>=LONG_MAX){ // solo per adesso e necessario questo filtro..
        return -2;
    }
    if((i->second)->selectAxis()<0){
        return -3;
    }

    if((i->second)->moveAbsoluteSteps((long)nMicroSteps)<0){
        return -4;
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

int ActuatorTechnoSoft::getPosition(int axisID,readingTypes mode, double* deltaPosition_mm){
    DPRINT("Position reading, axisID %d",axisID);

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

    if((i->second)->selectAxis()<0){
        return -2;
    }

    if(mode==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
        //long tposition;
        if((i->second)->getCounter(deltaPosition_mm)<0){
            DERR("getting counter");
            return -3;
        }
        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
        //*deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*N_ROUNDS_DEFAULT);
    }
    else if(mode==READ_ENCODER){ // Lettura posizione per mezzo dell'encoder (Apos register)
        //long aposition;
        if((i->second)->getEncoder(deltaPosition_mm)<0){
            return -4;
        }
        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
        //*deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(N_ENCODER_LINES_DEFAULT*N_ROUNDS_DEFAULT);
    }
    return 0;
}

int ActuatorTechnoSoft::homing(int axisID,homingType mode){

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

    if((i->second)->selectAxis()<0){
        return -2;
    }
    return ((i->second)->homing(mode));
}



int ActuatorTechnoSoft::getState(int axisID,int* state, std::string& descStr){

    // ******************** NOTA:  le funzioni getState e getAlarms devono essere eseguite in modalita' SEQUENZIALE,
    // e non indipendentemente l'una dall'altra in thread differenti, perche' accedono agli stessi dati *********************

    DPRINT("Getting state of the actuator. ");

    *state  = ACTUATOR_UNKNOWN_STATUS;
    descStr.assign("");

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

    int stCode=0;

    uint16_t contentRegSRH; // remember typedef uint16_t WORD;
    uint16_t contentRegSRL;

    if((i->second)->selectAxis()<0){
        return -2;
    }

    (i->second)->stateInfoRequest = true; // Comunico al motore che sto richiedendo info riguardanti gli allarmi


    short indexReg = 4; // see constant REG_SRH in TML_lib.h
    (i->second)->regSRHrequest = true;
    if(((i->second)->getStatusOrErrorReg(indexReg, contentRegSRH, descStr))<0){
        ERR("Reading state error: %s",descStr.c_str());
        return -3;
    }
    (i->second)->regSRHrequest = false;

//    indexReg = 3; // see constant REG_SRL in TML_lib.h
//    (i->second)->regSRLrequest = true;
//    if(((i->second)->getStatusOrErrorReg(indexReg, contentRegSRL, descStr))<0){
//        ERR("Reading state error: %s",descStr.c_str());
//        return -4;
//    }
//    (i->second)->regSRLrequest = false;
    
    if((i->second)->readyState){ // readyState = true se la procedura di inizializzazione è andata a buon fine. Accendo il primo bit
        stCode|=ACTUATOR_READY;
        descStr=descStr+"Ready. ";
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
    if(contentRegSRH & ((uint16_t)1<<15)){ // **************** Per adesso lasciamolo cosi il fault status *********************
        stCode |= ACTUATOR_FAULT;
        descStr+="Fault status. ";
    }
    //  Analysis of the register content SRL
//    if(!(contentRegSRL & ((uint16_t)1<<10))){
//        stCode |= ACTUATOR_INMOTION;
//        descStr+="Actuator in motion.";
//    }                                                                  
    if((i->second)->actuatorIDInMotion){
        stCode |= ACTUATOR_INMOTION;
        descStr+="Actuator in motion.";
    }
//    if(contentRegSRL & ((uint16_t)1<<15)){
//        stCode |= ACTUATOR_POWER_SUPPLIED;
//        descStr += "Electrical power supplied.";
//    }
    
    if((i->second)->powerOffCommand){
        stCode |= ACTUATOR_POWER_SUPPLIED;
        descStr += "Electrical power supplied.";
    }
    // Homing in progress state
    if((i->second)->internalHomingStateDefault>0 || (i->second)->internalHomingStateHoming2>0){
        stCode |= HOMING_IN_PROGRESS;
        descStr += "Homing in progress.";
    }
    
    (i->second)->stateInfoRequest = false;
    *state = stCode;
    return 0;
}

int ActuatorTechnoSoft::getAlarms(int axisID, uint64_t* alrm, std::string& descStr){

    DPRINT("Getting alarms of the actuator");

    * alrm = ACTUATOR_NO_ALARMS_DETECTED;
    descStr.assign("");

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

    int stCode=0;

    uint16_t contentRegMER; // remember typedef uint16_t WORD;
    uint16_t contentRegSRH;

    if((i->second)->selectAxis()<0){
        return -2;
    }

    (i->second)->alarmsInfoRequest = true; // Comunico al motore che sto richiedendo info riguardanti gli allarmi

    short indexRegMER = 5; // see constant REG_MER in TML_lib.h
    (i->second)->regMERrequest = true;
    if((i->second)->getStatusOrErrorReg(indexRegMER, contentRegMER, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        return -3;
    }
    (i->second)->regMERrequest = false;

    short indexRegSRH = 4; // see constant REG_SRH in TML_lib.h
    (i->second)->regSRHrequest = true;
    if((i->second)->getStatusOrErrorReg(indexRegSRH, contentRegSRH, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        return -4;
    }
    (i->second)->regSRHrequest = false;

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
//            else if(i==6){
//                stCode|=ACTUATOR_LSP_LIMIT_ACTIVE;
//                descStr+="Positive limit switch active. ";
//            }
//            else if(i==7){
//                stCode|=ACTUATOR_LSN_LIMIT_ACTIVE;
//                descStr+="Negative limit switch active. ";
//            }
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

    
    // STATO LIMIT SWITCHES
    if((i->second)->LSPactive){
        stCode|=ACTUATOR_LSP_LIMIT_ACTIVE;
        descStr+="Positive limit switch active. ";
    }
    if((i->second)->LSNactive){
        stCode|=ACTUATOR_LSN_LIMIT_ACTIVE;
        descStr+="Negative limit switch active. ";
    }

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
    (i->second)->alarmsInfoRequest = false; // Comunico al motore che ho terminato di richiedere info riguardanti gli allarmi

    return 0;
}

int ActuatorTechnoSoft::stopMotion(int axisID){

    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }

     if((i->second)->selectAxis()<0){
        return -2;
     }
     if((i->second)->stopMotion()<0){
        return -3;
     }
     return 0;
 }

 int ActuatorTechnoSoft::resetAlarms(int axisID,uint64_t mode){

     // In the fault status the power stage is disabled, the MER register signals
     // the errors occurred and  bit 15 from the SRH is set to high to signal the fault state
     // ************************** Operazione di selezione axisID ***************************
     std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
     // Controlliamo comunque se l'axis id e' stato configurato
     if(i==motors.end()){
         // In questo caso il motore axisID non e' stato configurato
         return -1;
     }

     int err = 0;
     if((i->second)->selectAxis()<0){
        return -2;
     }
     mode =0;
     switch(mode){
         case 0:
            if((i->second)->resetFault()<0){
                err = -3;
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
             err = -4;
             break;
         default:
             err = -5;
             break;
     }
     return err;
 }

 int ActuatorTechnoSoft::getSWVersion(int axisID, std::string& version){

     std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
     // Controlliamo comunque se l'axis id e' stato configurato
     if(i==motors.end()){
         // In questo caso il motore axisID non e' stato configurato
         return -1;
     }
     if((i->second)->selectAxis()<0){
        return -2;
     }
     char firmVers[100];
     if((i->second)->getFirmwareVers(&firmVers[0])<0){
        version = "No firmware version retrivied";
        return -3;
     }
     version.assign(firmVers);
     return 0;
 }


int ActuatorTechnoSoft::poweron(int axisID,int on){

     // In the fault status the power stage is disabled, the MER register signals
     // the errors occurred and  bit 15 from the SRH is set to high to signal the fault state
     // ************************** Operazione di selezione axisID ***************************
     std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
     // Controlliamo comunque se l'axis id e' stato configurato
     if(i==motors.end()){
         // In questo caso il motore axisID non e' stato configurato
         return -1;
     }

     if((i->second)->selectAxis()<0){
         return -2;
     }
     int resp;
     switch (on) {
         case 0:
             if((i->second)->stopPower()<0){
                 resp=-3;
             }
             resp=0;
             break;
         case 1:
             if((i->second)->providePower()<0){
                 resp=-4;
             }
             resp=0;
             break;
         default:
             resp=-5;
             break;
     }
     return resp;
}

int ActuatorTechnoSoft::getHWVersion(int axisID, std::string& version){
     std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
     // Controlliamo comunque se l'axis id e' stato configurato
     if(i==motors.end()){
         // In questo caso il motore axisID non e' stato configurato
         return -1;
     }
     if((i->second)->selectAxis()<0){
        return -2;
     }
    version=" Technosoft IDM 240 stepper open loop mode";
    return 0;
}

int ActuatorTechnoSoft::sendDataset(std::string& dataset){
   dataset.clear();
   dataset="{\"attributes\":[";
   dataset+="{\"name\":\"speed\",\"description\":\"Max speed of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"500.0\",\"default\":\"400.0\"},";
   dataset+="{\"name\":\"acceleration\",\"description\":\"Acceleration of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"2.0\",\"default\":\"0.6\"},";
   dataset+="{\"name\":\"isadditive\",\"description\":\"Specifies how is computed the position to reach\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"0\"},";
   dataset+="{\"name\":\"movement\",\"description\":\"Defines the moment when the motion is started\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"-1\",\"max\":\"1\",\"default\":\"1\"},";
   dataset+="{\"name\":\"referenceBase\",\"description\":\"Specifies how the motion reference is computed\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"1\"},";
   dataset+="{\"name\":\"highspeedhoming\",\"description\":\"Max speed of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"15.0\",\"default\":\"10.0\"},";
   dataset+="{\"name\":\"lowspeedhoming\",\"description\":\"Speed of trapezoidal profile for homing procedure, for repositioning slit at LSN switch\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"3.0\",\"default\":\"1.0\"},";
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
