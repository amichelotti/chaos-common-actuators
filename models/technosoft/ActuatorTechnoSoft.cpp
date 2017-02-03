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

std::map<int,TechnoSoftLowDriver *> ActuatorTechnoSoft::motors;

ActuatorTechnoSoft::ActuatorTechnoSoft(){
    //driver=NULL;
    channel = NULL;
    initChannelAlreadyDone = false;
    //configAxisAlreadyDone = false,
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
    
//    if(initChannelAlreadyDone){
//        DPRINT("This object has already a communication channel correctly initialized");
//        return 0;
//    }
    
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
            ERR("Cannot possible init channel because value of channel is NULL");
            return -2;
        }
        if(channel->open()<0){
            ERR("Cannot possible open channel");
            return -3;
        }
       
        //initChannelAlreadyDone = true;
        
        // Inizializzazione mutex
        pthread_mutex_init(&(mu),NULL);
        delectingActuator = false;
        
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
        if(pthread_mutex_lock(&(mu))!=0){

        }
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
                if(pthread_mutex_unlock(&(mu))!=0){

                }
                return -2;
            }
            if(pthread_mutex_unlock(&(mu))!=0){

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

int ActuatorTechnoSoft::hardreset(int axisID, bool mode){ 

    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){ 
        // In questo caso il motore axisID non e' stato configurato, non c'e' quindi alcun motore da inizializzare
        return -1;
    }
    
    if((i->second)->selectAxis()<0){
        return -2;
    }
    if((i->second)->hardreset(mode)<0){
        return -3;
    }
    return 0;
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
//    if((i->second)->selectAxis()<0){
//        return -2;
//    }    
//    if((i->second)->stopMotion()<0){
//        return -3;
//    }   
//    if((i->second)->stopPower()<0){
//        return -4;
//    }    
    
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

int ActuatorTechnoSoft::getParameter(int axisID,std::string parName,std::string& resultString){
    
    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){ 
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }
    
    DPRINT("getParameter execution");
    
    // trim
    trim2(parName);
    // To upper case
    std::string strResultparName;
    setUpperCase(parName,strResultparName);
    
    DPRINT("Stringa elaborata %s",strResultparName.c_str());
    
    double doubleValue;
    int intValue;
//    
    if(strResultparName.compare("SPEED")==0){ 
        if((i->second)->getSpeed(doubleValue)<0){
            DPRINT("getParameter execution error");
            return -2;
        }
        // 1. Conversione valore numerico ---> Stringa
        
        // 2. resultString =  nuova_stringa_convertita 
        
        return 0;
    } 
    else if(strResultparName.compare("MAXSPEED")==0){ 
        if((i->second)->getMaxSpeed(doubleValue)<0){
            DPRINT("getParameter execution error");
            return -3;
        }
        // 1. Conversione valore numerico ---> Stringa
        
        // 2. resultString =  nuova_stringa_convertita 
        
        return 0;
    }  
    else if(strResultparName.compare("ACCELERATION")==0){
        if((i->second)->getAcceleration(doubleValue)<0){ 
            DPRINT("getParameter execution error");
            return -4;
        }
        // 1. Conversione valore numerico ---> Stringa
        
        // 2. resultString =  nuova_stringa_convertita
        
        return 0;
    }
    else if(strResultparName.compare("ISADDITIVE")==0){
        // Conversion from string to bool
        //boolValue = to_bool(valueOfparName);
        if((i->second)->getAdditive(intValue)<0){ 
            return -5;
        }
        // 1. Conversione valore numerico ---> Stringa
        
        // 2. resultString =  nuova_stringa_convertita
        
        return 0;
    }
//    else if(strResultparName.compare("MOVEMENT")==0){
//        if((i->second)->getMovement((short)intValue)<0){ 
//            return -6;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;
//    }
//    else if(strResultparName.compare("REFERENCEBASE")==0){
//        if((i->second)->getReferenceBase((short)intValue)<0){ 
//            return -7;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;   
//    }
//    else if(strResultparName.compare("HIGHSPEEDHOMING")==0){
//        if((i->second)->gethighSpeedHoming(doubleValue)<0){ 
//            return -8;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;       
//    }
//    else if(strResultparName.compare("MAXHIGHSPEEDHOMING")==0){
//        if((i->second)->getMaxhighSpeedHoming(doubleValue)<0){ 
//            return -9;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;       
//    }
//    else if(strResultparName.compare("LOWSPEEDHOMING")==0){
//        if((i->second)->getlowSpeedHoming(doubleValue)<0){ 
//            return -10;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;   
//    }
//    else if(strResultparName.compare("MAXLOWSPEEDHOMING")==0){
//        if((i->second)->getMaxlowSpeedHoming(doubleValue)<0){ 
//            return -11;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;   
//    }
//    else if(strResultparName.compare("ACCELERATIONHOMING")==0){
//        if((i->second)->getaccelerationHoming(doubleValue)<0){ 
//            return -12;
//        }
//        // 1. Conversione valore numerico ---> Stringa
//        
//        // 2. resultString =  nuova_stringa_convertita
//        
//        return 0;  
//    }
    
    return 0;
}

int ActuatorTechnoSoft::setParameter(int axisID,std::string parName,std::string valueOfparName){
    
    // ************************** Operazione di selezione axisID ***************************
    std::map<int,TechnoSoftLowDriver* >::iterator i = motors.find(axisID);
    // Controlliamo comunque se l'axis id e' stato configurato
    if(i==motors.end()){ 
        // In questo caso il motore axisID non e' stato configurato
        return -1;
    }
    
    DPRINT("setParameter execution");
    
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
    
    DPRINT("Stringa elaborata %s",strResultparName.c_str());
    
    if(strResultparName.compare("SPEED")==0){ 
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setSpeed(doubleValue)<0){
            DPRINT("setParameter execution error");
            return -2;
        }
        
        return 0;
    }   
    else if(strResultparName.compare("MAXSPEED")==0){ 
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setMaxSpeed(doubleValue)<0){
            DPRINT("setParameter execution error");
            return -3;
        }
        
        return 0;
    }  
    else if(strResultparName.compare("ACCELERATION")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setAcceleration(doubleValue)<0){ 
            return -4;
        }
        return 0;
    }
    else if(strResultparName.compare("MAXACCELERATION")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setMaxAcceleration(doubleValue)<0){ 
            return -5;
        }
        return 0;
    }
    else if(strResultparName.compare("ISADDITIVE")==0){
        // Conversion from string to bool
        //boolValue = to_bool(valueOfparName);
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setAdditive(intValue)<0){ 
            return -5;
        }
        return 0;
    }
    else if(strResultparName.compare("MOVEMENT")==0){
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setMovement((short)intValue)<0){ 
            return -6;
        }
        return 0;
    }
    else if(strResultparName.compare("REFERENCEBASE")==0){
        intValue = atoi(valueOfparName.c_str());
        if((i->second)->setReferenceBase((short)intValue)<0){ 
            return -7;
        }
        return 0;   
    }
    else if(strResultparName.compare("HIGHSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->sethighSpeedHoming(doubleValue)<0){ 
            return -8;
        }
        return 0;       
    }
    else if(strResultparName.compare("MAXHIGHSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setMaxhighSpeedHoming(doubleValue)<0){ 
            return -9;
        }
        return 0;       
    }
    else if(strResultparName.compare("LOWSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setlowSpeedHoming(doubleValue)<0){ 
            return -10;
        }
        return 0;   
    }
    else if(strResultparName.compare("MAXLOWSPEEDHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setMaxlowSpeedHoming(doubleValue)<0){ 
            return -11;
        }
        return 0;   
    }
    else if(strResultparName.compare("ACCELERATIONHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setaccelerationHoming(doubleValue)<0){ 
            return -12;
        }
        return 0;   
    }
    else if(strResultparName.compare("MAXACCELERATIONHOMING")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setMaxAccelerationHoming(doubleValue)<0){ 
            return -13;
        }
        return 0;   
    }
//    else if(strResultparName.compare("ISADDITIVEHOMING")==0){
//        // Conversion from string to bool
//        //boolValue = to_bool(valueOfparName);
//        intValue = atoi(valueOfparName.c_str());
//        if((i->second)->setAdditiveHoming(intValue)<0){ 
//            return -14;
//        }
//        return 0;
//    }
//    else if(strResultparName.compare("MOVEMENTHOMING")==0){
//        intValue = atoi(valueOfparName.c_str());
//        if((i->second)->setMovementHoming((short)intValue)<0){ 
//            return -15;
//        }
//        return 0;
//    }
//    else if(strResultparName.compare("REFERENCEBASEHOMING")==0){
//        intValue = atoi(valueOfparName.c_str());
//        if((i->second)->setReferenceBaseHoming((short)intValue)<0){ 
//            return -16;
//        }
//        return 0;   
//    }//_________________________________________________________________________
    else if(strResultparName.compare("NUMENCODERLINES")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setEncoderLines(doubleValue)<0){ 
            return -17;
        }
        return 0;   
    }
    else if(strResultparName.compare("NUMMICROSTEPSPERSTEP")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setConst_mult_technsoft(doubleValue)<0){ 
            return -18;
        }
        return 0;   
    } 
    else if(strResultparName.compare("STEPSPERROUND")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setSteps_per_rounds(doubleValue)<0){ 
            return -19;
        }
        return 0;   
    } 
    else if(strResultparName.compare("FIXEDNUMBEROFROUNDS")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setN_rounds(doubleValue)<0){ 
            return -20;
        }
        return 0;   
    } 
    else if(strResultparName.compare("LINEARDISPLACEMENT[MM]")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setLinear_movement_per_n_rounds(doubleValue)<0){ 
            return -21;
        }
        return 0;   
    } 
    else if(strResultparName.compare("VOLTAGE_LNS[V]")==0){ 
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setvoltage_LNS(doubleValue)<0){ 
            return -22;
        }
        return 0;   
    } 
    else if(strResultparName.compare("VOLTAGE_LPS[V]")==0){
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setvoltage_LPS(doubleValue)<0){ 
            return -23;
        }
        return 0;   
    } 
    else if(strResultparName.compare("RANGE_SLIT[MM]")==0){  //range_slit[mm]
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setRange(doubleValue)<0){ 
            return -24;
        }
        return 0;   
    }
    else if(strResultparName.compare("FULLSCALEPOT")==0){ //fullscalePot
        doubleValue = atof(valueOfparName.c_str());
        if((i->second)->setFullscalePot(doubleValue)<0){ 
            return -25;
        }
        return 0;   
    }
    
    else{
        return -26;
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

//int ActuatorTechnoSoft::setTrapezoidalProfile(double speed, double acceleration, bool isAdditive, int32_t movement, int32_t referenceBase){
//    
//    if(driver->setSpeed(speed)<0){
//        return -1;
//    }
//    if(driver->setAcceleration(acceleration)<0){
//        return -2;
//    }
//    if(driver->setAdditive(isAdditive)<0){
//        return -3;
//    }
//    if(driver->setMovement(movement)<0){
//        return -4;
//    }
//    if(driver->setReferenceBase(referenceBase)<0){
//        return -5;
//    }
//    return 0;
//}

//int ActuatorTechnoSoft::setSpeed(double speed){
//    if(driver->setSpeed(speed)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMaxSpeed(double speed){
//    if(driver->setMaxSpeed(speed)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setAcceleration(double acceleration){
//    if(driver->setAcceleration(acceleration)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMaxAcceleration(double maxAcceleration){
//    if(driver->setMaxAcceleration(maxAcceleration)<0){
//        return -1;
//    }
//    return 0;
//}
//
//
//int ActuatorTechnoSoft::setAdditive(bool isAdditive){
//    if(driver->setAdditive((int)isAdditive)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMovement(int32_t movement){
//    if(driver->setMovement((short)movement)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setReferenceBase(int32_t referenceBase){
//    if(driver->setReferenceBase((short)referenceBase)<0){
//        return -1;
//    }                
//    return 0;
//}
//
//// Set homing parameters
//int ActuatorTechnoSoft::sethighSpeedHoming(double speed){
//    if(driver->sethighSpeedHoming(speed)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMaxhighSpeedHoming(double maxHighSpeed){
//    if(driver->setMaxhighSpeedHoming(maxHighSpeed)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setlowSpeedHoming(double speed){
//    if(driver->setlowSpeedHoming(speed)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMaxlowSpeedHoming(double speed){
//    if(driver->setMaxlowSpeedHoming(speed)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setAccelerationHoming(double acceleration){
//    if(driver-> setaccelerationHoming(acceleration)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMaxAccelerationHoming(double maxacceleration){
//    if(driver-> setMaxAccelerationHoming(maxacceleration)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setAdditiveHoming(bool isAdditive){
//    if(driver->setAdditiveHoming((int)isAdditive)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setMovementHoming(int32_t movement){
//    if(driver->setMovementHoming((short)movement)<0){
//        return -1;
//    }
//    return 0;
//}
//
//int ActuatorTechnoSoft::setReferenceBaseHoming(int32_t referenceBase){
//    if(driver->setReferenceBaseHoming((short)referenceBase)<0){
//        return -1;
//    }                
//    return 0;
//}
//
//int ActuatorTechnoSoft::setEncoderLines(double _encoderLines){
//    if(driver->setEncoderLines(_encoderLines)<0){
//        return -1;
//    }   
//    return 0;
//}
//
//int ActuatorTechnoSoft::setConst_mult_technsoft(double _const_mult_technsoft){
//    if(driver->setConst_mult_technsoft(_const_mult_technsoft)<0){
//        return -1;
//    }    
//    return 0;
//}
//
//int ActuatorTechnoSoft::setSteps_per_rounds(double _steps_per_rounds){
//    if(driver->setSteps_per_rounds(_steps_per_rounds)<0){
//        return -1;
//    }    
//    return 0;
//}
//
//int ActuatorTechnoSoft::setN_rounds(double _n_rounds){
//    if(driver->setN_rounds(_n_rounds)<0){
//        return -1;
//    }   
//    return 0;
//}
//
//int ActuatorTechnoSoft::setLinear_movement_per_n_rounds(double _linear_movement_per_n_rounds){
//    if(driver->setLinear_movement_per_n_rounds(_linear_movement_per_n_rounds)<0){
//        return -1;
//    }   
//    return 0;
//}
           
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

//    if(mode==READ_COUNTER){ // Lettura posizione per mezzo del counter (TPOS register)
//        //long tposition;
//        if((i->second)->getCounter(deltaPosition_mm)<0){
//            DERR("getting counter");
//            return -3;
//        }
//        //std::cout<< "Il valore del counter e':"<<tposition <<std::endl;
//        //*deltaPosition_mm = (tposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*N_ROUNDS_DEFAULT);
//    }
//    else if(mode==READ_ENCODER){ // Lettura posizione per mezzo dell'encoder (Apos register)
//        //long aposition;
//        if((i->second)->getEncoder(deltaPosition_mm)<0){
//            return -4;
//        }
//        //std::cout<< "Il valore dell'encoder e':"<<aposition <<std::endl;
//        //*deltaPosition_mm = (aposition*LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT)/(N_ENCODER_LINES_DEFAULT*N_ROUNDS_DEFAULT);
//    }
//    else if(mode==READ_POTENTIOMETER){
//        if((i->second)->getPotentiometer(deltaPosition_mm)<0){
//            return -5;
//        }
//    }
    switch(mode)
    {
    case (READ_COUNTER):
         if((i->second)->getCounter(deltaPosition_mm)<0){
            DERR("getting counter");
            return -3;
        }
        break;
            case (READ_ENCODER):
            if((i->second)->getEncoder(deltaPosition_mm)<0){
                return -4;
            }
        break;
            case (READ_POTENTIOMETER):
            if((i->second)->getPotentiometer(deltaPosition_mm)<0){
                return -5;
            }
            break;
        default:   
            break;
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
    uint16_t contentRegMER;
    
    if((i->second)->selectAxis()<0){
        return -2;
    }

    short indexReg = 4; // see constant REG_SRH in TML_lib.h
    if(((i->second)->getStatusOrErrorReg(indexReg, contentRegSRH, descStr))<0){
        stCode|=ACTUATOR_UNKNOWN_STATUS;
        descStr=descStr+"Unknown status. ";
        ERR("Reading state error: %s",descStr.c_str());
        return -3;
    }
    indexReg = 3; // see constant REG_SRL in TML_lib.h
    if(((i->second)->getStatusOrErrorReg(indexReg, contentRegSRL, descStr))<0){
        stCode|=ACTUATOR_UNKNOWN_STATUS;
        descStr=descStr+"Unknown status. ";
        ERR("Reading state error: %s",descStr.c_str());
        return -4;
    }
    indexReg = 5;
    if(((i->second)->getStatusOrErrorReg(indexReg, contentRegMER, descStr))<0){
        stCode|=ACTUATOR_UNKNOWN_STATUS;
        descStr=descStr+"Unknown status. ";
        ERR("Reading state error: %s",descStr.c_str());
        return -4;
    }

    if((i->second)->readyState){ // readyState = true se la procedura di inizializzazione è andata a buon fine. Accendo il primo bit
        stCode|=ACTUATOR_READY;
        descStr=descStr+"Ready. ";
    }
    
    if((contentRegSRH & ((uint16_t)1<<1)) || (contentRegSRH & ((uint16_t)1<<2)) || (contentRegSRH & ((uint16_t)1<<3)) || (contentRegSRH & ((uint16_t)1<<4))){ // readyState = true se la procedura di inizializzazione è andata a buon fine. Accendo il primo bit
        stCode|=ACTUATOR_OVER_POSITION_TRIGGER;
        descStr=descStr+"Over Position Trigger. ";
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
        stCode |= ACTUATOR_LSN_LIMIT_ACTIVE;
        descStr+="Limit switch negative event/interrupt. ";
    }
    
    if(contentRegSRH & ((uint16_t)1<<10)){
        stCode|=ACTUATOR_I2T_WARNING_MOTOR;
        descStr+="Motor I2T protection warning. ";
    }
    if(contentRegSRH & ((uint16_t)1<<11)){
        stCode|=ACTUATOR_I2T_WARNING_DRIVE;
        descStr+="Drive I2T protection warning";
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
    if(!(contentRegSRL & ((uint16_t)1<<10))){
        stCode |= ACTUATOR_INMOTION;
        descStr+="Actuator in motion.";
    }
    if(contentRegSRL & ((uint16_t)1<<15)){
        stCode |= ACTUATOR_POWER_SUPPLIED;
        descStr += "Electrical power supplied.";
    }
     
    // Homing in progress state
    if((i->second)->internalHomingStateDefault>0 || (i->second)->internalHomingStateHoming2>0){
        stCode |= HOMING_IN_PROGRESS;
        descStr += "Homing in progress.";
    }
    
    // Lettura stato limit switch
    if(contentRegMER & ((uint16_t)1<<6)){
        stCode|=ACTUATOR_LSP_LIMIT_ACTIVE;
        descStr+="Positive limit switch active. ";    
    }
    
    if(contentRegMER & ((uint16_t)1<<7)){
        stCode|=ACTUATOR_LSN_LIMIT_ACTIVE;
        descStr+="Negative limit switch active. ";
        
    }

    *state = stCode;
    return 0;
}

int ActuatorTechnoSoft::getAlarms(int axisID, uint64_t* alrm, std::string& descStr){

    DPRINT("Getting alarms of the actuator");
    
    * alrm = 0;
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
    
    short indexRegMER = 5; // see constant REG_MER in TML_lib.h
    if((i->second)->getStatusOrErrorReg(indexRegMER, contentRegMER, descStr)<0){
        stCode|=ACTUATOR_ALARMS_READING_ERROR;
        descStr+= "Reading alarms error. ";
        DERR("Reading alarms error: %s",descStr.c_str());
        return -3;
    }

    short indexRegSRH = 4; // see constant REG_SRH in TML_lib.h
    if((i->second)->getStatusOrErrorReg(indexRegSRH, contentRegSRH, descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        stCode|=ACTUATOR_ALARMS_READING_ERROR;
        descStr+= "Alarms reading error. ";
        return -4;
    }
    
    // Lettura eventuale stato di emergenza:
    //uint8_t numRegEmergency = 16;
    uint8_t emergengyState;
    if((i->second)->getEmergency(16,emergengyState,descStr)<0){
        DERR("Reading alarms error: %s",descStr.c_str());
        stCode|=ACTUATOR_ALARMS_READING_ERROR;
        descStr+= "Alarms reading error. ";
        return -5;
    }

    if(contentRegMER & ((uint16_t)1<<0)){
        stCode|=ACTUATOR_CANBUS_ERROR; // IMPORTANTE: ACTUATOR_CANBUS_ERROR è di tipo int (32 bit)
        // Nell'operazione di OR logico, automaticamente il contenuto
        // a destra dell'uguale viene prima memorizzato in una locazione
        // a 64 bit di tipo unsigned cosicché si possa fare l'OR logico
        // bit a bit con la variabile a primo membro
        // In corrispondenza di questo errore accendo il bit 0 di *alarm
        //desc.assign("CAN bus error. ");
        descStr=descStr+"CAN bus error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<1)){
    
        stCode|=ACTUATOR_SHORT_CIRCUIT; // In corrispondenza di questo errore accendo il bit 1 di *alarm
        descStr+="Short circuit. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<2)){
        stCode|=ACTUATOR_INVALID_SETUP_DATA;
        descStr+= "Invalid setup data. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<3)){
        stCode|=ACTUATOR_CONTROL_ERROR;
        descStr+= "Control error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<4)){
        stCode|=ACTUATOR_SERIAL_COMM_ERROR;
        descStr= descStr+ "Communication error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<5)){
        stCode|=ACTUATOR_HALL_SENSOR_MISSING;
        descStr+= "Hall sensor missing / Resolver error / BiSS error / Position wrap around error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<8)){
        stCode|=ACTUATOR_OVER_CURRENT;
        descStr+="Over current error. ";       
    }

    if(contentRegMER & ((uint16_t)1<<9)){
        stCode|=ACTUATOR_I2T;
        descStr+="I2T protection error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<10)){
        stCode|=ACTUATOR_OVERTEMP_MOTOR;
        descStr+="Motor over temperature error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<11)){
        stCode|=ACTUATOR_OVERTEMP_DRIVE;
        descStr+="Drive over temperature error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<12)){
        stCode|=ACTUATOR_OVERVOLTAGE;
        descStr+="Over voltage error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<13)){
        stCode|=ACTUATOR_UNDERVOLTAGE;
        descStr+="Under voltage error. ";
    }
    
    if(contentRegMER & ((uint16_t)1<<14)){
        stCode|=ACTUATOR_COMMANDERROR;
        descStr+="Command error. ";
    }
    
    if(!emergengyState){
        stCode|=ACTUATOR_ALARMS_EMERGENCY_ERROR;
        descStr+="Emergency error. ";
    }
     
    *alrm = stCode;
    
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
   dataset+="{\"name\":\"maxspeed\",\"description\":\"Maximum value for max speed of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"1000.0\",\"default\":\"700.0\"},";
   dataset+="{\"name\":\"acceleration\",\"description\":\"Acceleration of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"2.0\",\"default\":\"0.6\"},";
   dataset+="{\"name\":\"maxacceleration\",\"description\":\"Maximum value for acceleration of trapezoidal profile\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"5.0\",\"default\":\"0.9\"},";
   dataset+="{\"name\":\"isadditive\",\"description\":\"Specifies how is computed the position to reach\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"0\"},";
   dataset+="{\"name\":\"movement\",\"description\":\"Defines the moment when the motion is started\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"-1\",\"max\":\"1\",\"default\":\"1\"},";
   //dataset+="{\"name\":\"referenceBase\",\"description\":\"Specifies how the motion reference is computed\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"1\"},";
   dataset+="{\"name\":\"highspeedhoming\",\"description\":\"Max speed of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"15.0\",\"default\":\"10.0\"},";
   dataset+="{\"name\":\"maxhighspeedhoming\",\"description\":\"Maximum value for max speed of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"100.0\",\"default\":\"14.0\"},";
   dataset+="{\"name\":\"lowspeedhoming\",\"description\":\"Speed of trapezoidal profile for homing procedure, for repositioning slit at LSN switch\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"3.0\",\"default\":\"1.0\"},";
   dataset+="{\"name\":\"maxlowspeedhoming\",\"description\":\"Max value for speed of trapezoidal profile for homing procedure, for repositioning slit at LSN switch\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"6.0\",\"default\":\"4.0\"},"; 
   dataset+="{\"name\":\"accelerationhoming\",\"description\":\"Acceleration of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"0.6\",\"default\":\"0.3\"},";
   dataset+="{\"name\":\"maxaccelerationhoming\",\"description\":\"Max value for acceleration of trapezoidal profile for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.001\",\"max\":\"1.0\",\"default\":\"0.8\"},";
//   dataset+="{\"name\":\"isadditivehoming\",\"description\":\"Specifies how is computed the position to reach for homing procedure\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"0\"},";
//   dataset+="{\"name\":\"movementhoming\",\"description\":\"Defines the moment when the motion is started for homing procedure\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"-1\",\"max\":\"1\",\"default\":\"1\"},";
//   dataset+="{\"name\":\"referenceBaseHoming\",\"description\":\"Specifies how the motion reference is computed for homing procedure\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"1\"},";
   dataset+="{\"name\":\"numencoderlines\",\"description\":\"Number of encoder lines\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"800\"},";
   dataset+="{\"name\":\"nummicrostepsperstep\",\"description\":\"Number of micro steps per step\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"256\"},";
   dataset+="{\"name\":\"stepsperround\",\"description\":\"Number of steps to perfor a complete round\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"200\"},";
   dataset+="{\"name\":\"fixednumberofrounds\",\"description\":\"Number of rounds for which the linear displacement is known\",\"datatype\":\"int32\",\"direction\":\"Input\",\"min\":\"1\",\"max\":\"10000000\",\"default\":\"20\"},";
   dataset+="{\"name\":\"lineardisplacement[mm]\",\"description\":\"Linear displacement [mm] performed by slit associated with fixednumberofrounds rounds\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.000000001\",\"max\":\"10000000\",\"default\":\"1.5\"},";
   //dataset+="{\"name\":\"ratiOfNoise\",\"description\":\"Ratio of the real position rp in millimeter that identifies min and max values of pseudo white noise added to rp\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"1\",\"default\":\"0.0\"}";
   dataset+="{\"name\":\"voltage_LNS[V]\",\"description\":\"Voltage associated with LNS [V]\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"10000000000\",\"default\":\"7.7\"},";
   dataset+="{\"name\":\"voltage_LPS[V]\",\"description\":\"Voltage associated with LPS [V]\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0\",\"max\":\"10000000000\",\"default\":\"0.3\"},";
   dataset+="{\"name\":\"range_slit[mm]\",\"description\":\"Maximum linear displacement of the slit [m]\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.000000001\",\"max\":\"10000000000\",\"default\":\"10.0\"},";
   dataset+="{\"name\":\"fullscalePot\",\"description\":\"Full scale of the potentiometer\",\"datatype\":\"double\",\"direction\":\"Input\",\"min\":\"0.000000001\",\"max\":\"10000000000\",\"default\":\"20.0\"}";
   dataset+="]}";
   return 0;
}
