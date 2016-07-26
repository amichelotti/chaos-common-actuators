
#include "TechnoSoftLowDriverSim.h"
#include <common/debug/core/debug.h>
#include <iostream>

using namespace common::actuators::models;

//--------------------------------------------
void ElectricPowerException::badElectricPowerInfo(){
    
    std::cerr<< "The electrical power has not been turned off." << std::endl; 
}

void StopMotionException::badStopMotionInfo(){
    
    std::cerr<< "The eventual motion can not be stopped" << std::endl; 
}

void OpeningChannelException::badOpeningChannelInfo(){
    
    std::cerr<< "Channel can not be opened. " << std::endl; 
}

SerialCommChannelTechnosoft::SerialCommChannelTechnosoft(int hostID, const std::string& pszDevName,BYTE btType,DWORD baudrate){
    init(hostID, pszDevName,btType,baudrate); // La funzione init non ritorna mai un numero negativo per quello che fa, quindi 
                                       // e' inutile mettere il controllo. E poi se ritornasse un numero negativo bisognerebbe lanciare 
                                      // una eccezione dato che si tratta di un metodo costruttore
}

std::string  SerialCommChannelTechnosoft::getDevName(){return this->pszDevName;}
int SerialCommChannelTechnosoft::getbtType(){return this->btType;}
int SerialCommChannelTechnosoft::getbaudrate(){return this->baudrate;}

int SerialCommChannelTechnosoft::init(int _hostID, const std::string& _pszDevName,const BYTE& _btType,const DWORD& _baudrate){
    DPRINT("initializing dev %s type %d baud %d",_pszDevName.c_str(),_btType,_baudrate);
    pszDevName=_pszDevName;
    btType = _btType;
    baudrate = _baudrate;
    hostID = _hostID;
    fd = -1;
    return 0;
}

void SerialCommChannelTechnosoft::close(){

    if(fd!=-1){
	TS_CloseChannel(fd); // chiusura canale di comunicazione
    }
    DPRINT("Chiusura canale di comunicazione in fase di deallocazione delle risorse");
}

SerialCommChannelTechnosoft::~SerialCommChannelTechnosoft(){
    close();
    DPRINT("Deallocazione oggetto SerialCommChannelTechnosoft. Invio comando di chiusura canale di comunicazione effettuato");
}

int SerialCommChannelTechnosoft::open(){
    int resp;
    /*	Open the comunication channel: COM1, RS232, 1, 115200 */
    DPRINT("opening dev %s type %d baud %d, hostid:%d",pszDevName.c_str(),btType,baudrate,hostID);
    resp=TS_OpenChannel(pszDevName.c_str(), btType, hostID, baudrate);
    if(resp < 0){
      ERR("failed opening channel dev:%s type:%d host:%d baudrate: %d",pszDevName.c_str(), btType, hostID, baudrate);
      return -1;
    }
    this->fd = resp; 
    DPRINT("Openchannel file descriptor=%d",resp);
    return 0;
}

//TechnoSoftLowDriver::channel_map_t TechnoSoftLowDriver::channels; // Anche se non viene inizializzato...

//----------------------------------------------
TechnoSoftLowDriver::TechnoSoftLowDriver(){
    
//    // Inizializzazione membri privati
//    //alreadyopenedChannel = false;
//    //channelJustOpened = false;
//    //poweron = false;
//    //devName = _devName;
//    
//    if(my_channel==NULL)
//        DPRINT("In effetti lo shared pointer solo dichiarato ha valore NULL");
//    
//    channel_map_t::iterator i=channels.find(devName); // iteratore alla mappa statica
//    DPRINT("devName %s", devName.c_str());                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
//    
//    DPRINT("Dimensione mappa statica all'inizio prima di controllarla: %d",channels.size());
////    if(channels.size()>0){
////         DPRINT("Dimensione mappa statica: %d",channels.size());   
////    }
//    
//    if(i!=channels.end()){
//        my_channel = i->second;
//        // In questo caso non dovrò più provare ad aprire il canale di comunicazione
//        // nella sucessiva procedura di inizializzazione  del canale + drive/motor
//        // Settiamo dunque a true questo stato
//        DPRINT("Il canale e' gia' stato aperto, con baudrate = %d ", my_channel->getbaudrate());
//        
//        //alreadyopenedChannel = true;
//        // Verrà sfruttato il canale di comunicazione desiderato che è correntemente aperto
//        DPRINT("Channel %s, with name %s has been already opened", devName.c_str());
//    } 
//    else {
//        my_channel = channel_psh(new SerialCommChannelTechnosoft(hostID, devName, btType, baudrate));//********Altri due parametri OPZIONALI: const BYTE btType,const DWORD baudrate*******
//        //*****Nota il nuovo canale creato deve essere inserito nella mappa:
//        //channels.insert( std::pair<std::string,channel_psh>(devName,my_channel)); // IPOTESI: QUESTA FUNZIONE NON GENERA MAI ECCEZIONE 
//        if((my_channel->open()<0)){
//            DERR("error opening channel");
//            //return -23; // Attenzione: qui dovra' essere generata una eccezione!!!!
//            throw OpeningChannelException();
//        }
//        //channelJustOpened = true;
//        //if(channelJustOpened){
//        DPRINT("Inserimento elemento mappa: devName=%s",devName.c_str());
//        channels.insert(std::pair<std::string,channel_psh>(devName,my_channel)); // IPOTESI TEMPORANEA: QUESTA FUNZIONE NON può GENERAre MAI ECCEZIONE
//        //alreadyopenedChannel=true;
//        //}
//        DPRINT("Dimensione mappa statica alla fine della configurazione: %d",channels.size());
//        //alreadyopenedChannel=true; // Canale di comunicazione aperto e inizializzazione driver/motor andata a buon fine
//        DPRINT("channel just opened");
//        DPRINT("created channel  %s", devName.c_str());
//    }
//    readyState = true;
//    internalHomingStateDefault=0;
//    internalHomingStateHoming2=0; 
    
}

TechnoSoftLowDriver::~TechnoSoftLowDriver(){
    //deinit();
    //DPRINT("Deallocazione oggetto TechnoSoftLowDriver");
}    
 
int TechnoSoftLowDriver::init(const std::string& setupFilePath,
                        const int& _axisID,
                        const double _speed_mm_s,
                        const double _maxSpeed_mm_s, 
                        const double _acceleration_mm_s2,
                        const double _maxAcceleration_mm_s2,
                        const BOOL _isAdditive, 
                        const short _moveMoment,
                        const short _referenceBase,
                        const double _highSpeedHoming_mm_s,
                        const double _lowSpeedHoming_mm_s,
                        const double _maxHighSpeedHoming_mm_s,
                        const double _maxLowSpeedHoming_mm_s,
                        const double _accelerationHoming_mm_s2,
                        const double _maxAccelerationHoming_mm_s2,
                        const BOOL _isAdditiveHoming,
                        const short _movementHoming,
                        const short _referenceBaseHoming,
                        const double _n_encoder_lines, 
                        const double _const_mult_technsoft, 
                        const double _steps_per_rounds,    
                        const double _n_rounds,            
                        const double _linear_movement_per_n_rounds){
    
    DPRINT("Inizializzazione parametri");
    
    if(_maxSpeed_mm_s<=0){
        return -1;
    }
    maxSpeed_mm_s=_maxSpeed_mm_s;
    //DPRINT("maxSpeed_mm_s = %f",maxSpeed_mm_s);
    
    if(_speed_mm_s<=0 || _speed_mm_s>_maxSpeed_mm_s){
        return -2;
    }
    speed_mm_s = _speed_mm_s; 
    //DPRINT("speed_mm_s = %f",speed_mm_s);
    
    if(_maxAcceleration_mm_s2<=0){
        return -3;
    }
    maxAcceleration_mm_s2=_maxAcceleration_mm_s2;
    //DPRINT("maxAcceleration_mm_s2 = %f",maxAcceleration_mm_s2);
    
    if(_acceleration_mm_s2<=0 || _acceleration_mm_s2>_maxAcceleration_mm_s2){
        return -4;
    }
    acceleration_mm_s2=_acceleration_mm_s2;
    //DPRINT("acceleration_mm_s2 = %f",acceleration_mm_s2);
    
    if(_isAdditive!=TRUE && _isAdditive!=FALSE){
        return -5;
    }  
    isAdditive=_isAdditive;
    //DPRINT("isAdditive = %d",isAdditive);
    
    if((_moveMoment!=UPDATE_NONE) && (_moveMoment!=UPDATE_IMMEDIATE) && (_moveMoment!=UPDATE_ON_EVENT)){
        return -6;
    }
    movement=_moveMoment;
    //DPRINT("movement = %d",movement);
    
    if((_referenceBase!=FROM_MEASURE) && _referenceBase!=FROM_REFERENCE){
        return -7;
    }
    referenceBase=_referenceBase;
    //DPRINT("referenceBase = %d",referenceBase);
    
    // ********************* Set homing parameters *********************
    if(_maxHighSpeedHoming_mm_s<=0){
        return -8;
    }   
    maxHighSpeedHoming_mm_s = -_maxHighSpeedHoming_mm_s; // N.B. Dalla tastiera verra' inserito un numero positivo, 
                                               // che poi solo all'interno del drive ne verra' considerato 
                                               // solamente il segno opposto
    
    if(_highSpeedHoming_mm_s<=0 || _highSpeedHoming_mm_s>_maxHighSpeedHoming_mm_s){
        return -9;
    }
    highSpeedHoming_mm_s = -_highSpeedHoming_mm_s;
    
    if(_maxLowSpeedHoming_mm_s<=0){
        return -10;
    }   
    maxLowSpeedHoming_mm_s = _maxLowSpeedHoming_mm_s; // Verra' considerato il solo valore positivo perche' utilizzato
                                            // nel moveAbsoluteHoming
    
    if((_lowSpeedHoming_mm_s<=0) || (_lowSpeedHoming_mm_s>_maxLowSpeedHoming_mm_s)){
        return -11;
    }
    lowSpeedHoming_mm_s=_lowSpeedHoming_mm_s;
    
    if(_maxAccelerationHoming_mm_s2<=0){
        return -12;
    }
    maxAccelerationHoming_mm_s2=_maxAccelerationHoming_mm_s2;
    
    if(_accelerationHoming_mm_s2<=0 || _accelerationHoming_mm_s2>_maxAccelerationHoming_mm_s2){
        return -13;
    }
    accelerationHoming_mm_s2 = _accelerationHoming_mm_s2; 
    
    if(_isAdditiveHoming!=TRUE && _isAdditiveHoming!=FALSE){
        return -14;
    }  
    isAdditiveHoming=_isAdditiveHoming;
    
    if((_movementHoming!=UPDATE_NONE) && (_movementHoming!=UPDATE_IMMEDIATE) && (_movementHoming!=UPDATE_ON_EVENT)){
        return -15;
    }
    movementHoming=_movementHoming;  
    
    if((_referenceBaseHoming!=FROM_MEASURE) && _referenceBaseHoming!=FROM_REFERENCE){
        return -16;
    }
    referenceBaseHoming=_referenceBaseHoming;
    
    if(_n_encoder_lines<=0){
        return -17;
    }
    n_encoder_lines=_n_encoder_lines;
    
    if(_const_mult_technsoft<=0){
        return -19;
    }
    const_mult_technsoft=_const_mult_technsoft;
    
    if(_steps_per_rounds<=0){
        return -20;
    }
    steps_per_rounds=_steps_per_rounds;
    
    if(_n_rounds<=0){
        return -21;
    }
    n_rounds =_n_rounds;
    
    if(_linear_movement_per_n_rounds<=0){
        return -22;
    }
    linear_movement_per_n_rounds=_linear_movement_per_n_rounds;
    
    axisID = _axisID;
    
//    axisRef = TS_LoadSetup(setupFilePath.c_str());
//    if(axisRef < 0){
//        DERR("LoadSetup failed \"%s\"",setupFilePath.c_str());
//        return -24;
//    }
    std::srand(std::time(0));
    
    int random_variable = std::rand();
    if(random_variable<1*(RAND_MAX/20)) 
        return -24;
    
    /*	Setup the axis based on the setup data previously, for axisID*/
//    if(!TS_SetupAxis(_axisID, axisRef)){
//        DERR("failed to setup axis %d, %s",axisID,TS_GetLastErrorText());
//        return -25;
//    }
    random_variable = std::rand();
    if(random_variable<1*(RAND_MAX/20)) 
        return -25;
   
//    if(!TS_SelectAxis(_axisID)){
//        DERR("failed to select axis %d, %s",_axisID,TS_GetLastErrorText());
//        return -26;
//    }
    random_variable = std::rand();
    if(random_variable<1*(RAND_MAX/20)) 
        return -26;
    
    /*	Execute the initialization of the drive (ENDINIT) */
//    if(!TS_DriveInitialisation()){
//        DERR("failed Low driver initialisation");
//        return -27;
//    }
    random_variable = std::rand();
    if(random_variable<1*(RAND_MAX/20)) 
        return -27;
    
     // Settare il registro per la lettura dell'encoder
//    if(!TS_Execute("SCR=0x4338")){
//        //descrErr=descrErr+" "+TS_GetLastErrorText()+". ";
//        DERR("Failed TS_Execute command");
//        return -28;
//    }
    random_variable = std::rand();
    if(random_variable<1*(RAND_MAX/20)) 
        return -28;
  
//    if(!TS_SetEventOnMotionComplete(0,0)){ 
//	return -30;
//    }
    random_variable = std::rand();
    if(random_variable<1*(RAND_MAX/20)) 
        return -29;
    
    readyState = true;
    internalHomingStateDefault=0;
    internalHomingStateHoming2=0;
    cIP.position = 0;
    
    return 0;
}

//TechnoSoftLowDriver::channel_psh TechnoSoftLowDriver::getMyChannel(){
//    
//    return my_channel;
//}

int TechnoSoftLowDriver::homing(int mode){
    // Attenzione: la variabile mode non viene utilizzata
    
    if(mode==0){
        
        int risp;
        int switchTransited=0;
        int motionCompleted = 0;
        std::string cappos = "CAPPOS";
        long cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
        int absoluteMotionCompleted = 0;
    
        switch (internalHomingStateDefault) {
            case 0:
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -1;
                    break;
                }
                if(moveVelocityHoming()<0){
                    internalHomingStateDefault = 0;
                    if(stopMotion()<0){
                        risp = -2;
                        break;
                    }
                    risp = -3;
                    break;
                }
                DPRINT(" STATE 0: move velocity activated ");
                if(setEventOnLimitSwitch()<0){
                    internalHomingStateDefault = 0;
                    if(stopMotion()<0){
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
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -6;
                    break;
                }
                if(checkEvent(switchTransited)<0){
                    internalHomingStateDefault = 0;// deve essere riinizializzato per successivi nuovi tentativi di homing 
                    if(stopMotion()<0){
                        risp = -7;
                        break;
                    }    
                    risp = -8;
                    break;
                } 
                DPRINT(" STATE 1: possible limit switch transition just checked ");
                if(switchTransited){
                    if(setEventOnMotionComplete()<0){ 
                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
                        if(stopMotion()<0){
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
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -11;
                    break;
                }
                if(checkEvent(motionCompleted)<0){
                    internalHomingStateDefault = 0;
                    if(stopMotion()<0){
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
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -13;
                    break;
                }
                if(getLVariable(cappos, cap_position)<0){ 
    //                if(stopMotion()<0){
    //                    return -13;
    //                }
                    internalHomingStateDefault=0;
                    risp = -14;
                    break;
                }
                DPRINT("************** STATE 3: the captured position on limit switch transition is %ld [drive internal position units]**************",cap_position);
        
            /*	Command an absolute positioning on the captured position */
                if(moveAbsoluteStepsHoming(cap_position)<0){  
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
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -16;
                    break;
                }
//        if(!eventOnMotionCompleteSet){
//            if(setEventOnMotionComplete()<0){
//                if(stopMotion()<0){
//                    eventOnMotionCompleteSet = false;
//                    internalHomingStateDefault = 0;
//                    return -12;
//                }
//                eventOnMotionCompleteSet = false;
//                internalHomingStateDefault = 0;
//                return -13;
//            } 
//        }
                if(checkEvent(absoluteMotionCompleted)<0){
                    internalHomingStateDefault = 0;
                    if(stopMotion()<0){
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
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -19;
                    break;
                }
        
                if(resetEncoder()<0){
                    internalHomingStateDefault = 0;
                    //if(stopMotion()<0){
                    //return -23;
                    //}
                    risp= -20;
                    break;
                }
                if(resetCounter()<0){
                    internalHomingStateDefault = 0;
                    //if(stopMotion()<0){
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
    else if(mode==1){
        int risp;
        uint16_t contentReg;
        std::string descStr = "";
        short homingDone = 0;
        switch (internalHomingStateHoming2) {
            case 0:
                DPRINT("************** Homing procedure Homing2. STATE 0. **************");
                if(moveVelocityHoming()<0){
                    internalHomingStateHoming2=0;
                    risp= -1;
                    break;
                }
                internalHomingStateHoming2=1;
                risp= 1;
                break;
            case 1:
                DPRINT("************** Homing procedure Homing2. STATE 1. **************");
                if((getStatusOrErrorReg(5, contentReg, descStr))<0){
                    //ERR("Reading state error: %s",descStr.c_str());
                    internalHomingStateHoming2=0;
                    if(stopMotion()<0){
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
                if(resetEncoder()<0){
                    internalHomingStateHoming2=0;
                    if(stopMotion()<0){
                        risp= -4;
                        break;
                    }
                    risp= -5;
                    break;
                }
                if(resetCounter()<0){
                    internalHomingStateHoming2=0;
                    if(stopMotion()<0){
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

void* TechnoSoftLowDriver::incrDecrPosition(void* arg){
    
    // Stima grossolana tempo necessario per la movimentazione
    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
    // Hp: deltaPosition [micro step], speed_ms_s [micro step/s]
    // Intervallo di tempo necessario al completamento della movimentazione, espresso in secondi [s]
    // Dobbiamo tenere conto che in realta' il profilo di velocita' e' trapezoidale...
    deltaT += (deltaT*30/100);
    
    double totalTimeInterval = 0;   // solo per far partire il ciclo while
    struct timeval startTime,endTime;
    
    gettimeofday(&startTime,NULL);
    while(totalTimeInterval<=deltaT){
        // L'incremento deve avvenire ad una determinata velocita'
        if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
            
        }
            if(((containerIncrementPosition*)arg)->deltaPosition>=0)
                cIP.position+=speed_ms_s;
            else
                cIP.position-=speed_ms_s;
       
        if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
            
        }
        
        gettimeofday(&endTime,NULL);
        totalTimeInterval = ((double)endTime.tv_sec+(double)endTime.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
        sleep(1); // Sleep for 1 second   
    }
    pthread_exit(NULL);
}

int TechnoSoftLowDriver::moveRelativeSteps(const long& deltaPosition){
    
    DPRINT("Relative Moving axis: %d, deltaMicroSteps %d, speed=%f, acceleration %f, isadditive %d, movement %d, referencebase %d",axisID,deltaPosition,speed_mm_s,acceleration_mm_s2,isAdditive,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveRelative(deltaPosition, speed_mm_s, acceleration_mm_s2, isAdditive, movement, referenceBase)){
//        DERR("error relative moving");
//        return -2;
//    }
    int random_variable = std::rand();
    if(random_variable<(RAND_MAX/20)) 
        return -1;
    
    long cIP.deltaPosition = deltaPosition;
    pthread_t th;
    pthread_create(&th, NULL,incrDecrPosition,&cIP);
    
    return 0;
}

double TechnoSoftLowDriver::getdeltaMicroSteps(const double& deltaMillimeters){
    
    return round((steps_per_rounds*n_rounds*const_mult_technsoft*deltaMillimeters)/linear_movement_per_n_rounds);
}

void* TechnoSoftLowDriver::incrDecrPositionHoming(void* arg){
    
    // Stima grossolana tempo necessario per la movimentazione
    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/highSpeedHoming_mm_s); //[s]
    // Hp: deltaPosition [micro step], speed_ms_s [micro step/s]
    // Intervallo di tempo necessario al completamento della movimentazione, espresso in secondi [s]
    // Dobbiamo tenere conto che in realta' il profilo di velocita' e' trapezoidale...
    deltaT += (deltaT*30/100);
    
    double totalTimeInterval = 0;   // solo per far partire il ciclo while
    struct timeval startTime,endTime;
    
    gettimeofday(&startTime,NULL);
    while(totalTimeInterval<=deltaT){
        // L'incremento deve avvenire ad una determinata velocita'
        if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
            
        }
            if(((containerIncrementPosition*)arg)->deltaPosition>=0)
                cIP.position+=speed_ms_s;
            else
                cIP.position-=speed_ms_s;
       
        if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
            
        }
        
        gettimeofday(&endTime,NULL);
        totalTimeInterval = ((double)endTime.tv_sec+(double)endTime.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
        sleep(1); // Sleep for 1 second   
    }
    pthread_exit(NULL);
}

int TechnoSoftLowDriver::moveRelativeStepsHoming(const long& deltaPosition){
    DPRINT("Relative Moving axis: %d, deltaMicroSteps %d, speed=%f, acceleration %f, isadditive %d, movement %d, referencebase %d",axisID,deltaPosition,highSpeedHoming_mm_s,accelerationHoming_mm_s2,isAdditiveHoming,movementHoming,referenceBaseHoming);
    
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveRelative(deltaPosition, highSpeedHoming_mm_s, accelerationHoming_mm_s2, isAdditiveHoming, movementHoming, referenceBaseHoming)){
//        DERR("error relative moving homing");
//        return -2;
//    }
    DPRINT("Relative Moving axis: %d, deltaMicroSteps %d, speed=%f, acceleration %f, isadditive %d, movement %d, referencebase %d",axisID,deltaPosition,speed_mm_s,acceleration_mm_s2,isAdditive,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveRelative(deltaPosition, speed_mm_s, acceleration_mm_s2, isAdditive, movement, referenceBase)){
//        DERR("error relative moving");
//        return -2;
//    }
    int random_variable = std::rand();
    if(random_variable<(RAND_MAX/20)) 
        return -1;
    
    long cIP.deltaPosition = deltaPosition;
    pthread_t th;
    pthread_create(&th, NULL,incrDecrPositionHoming,&cIP);
    
    return 0;
}

// Set trapezoidal parameters
int TechnoSoftLowDriver::setSpeed(const double& _speed_mm_s){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    DPRINT("Chiamata setspeed");
    if(_speed_mm_s<=0 || _speed_mm_s>maxSpeed_mm_s){
        DERR("Speed = %f",_speed_mm_s);
        return -1;
    }
    speed_mm_s = _speed_mm_s;
    return 0;
}

int TechnoSoftLowDriver::setMaxSpeed(const double& _maxspeed_mm_s){
    
    if(_maxspeed_mm_s<=0 || _maxspeed_mm_s<speed_mm_s){
        DERR("Max speed = %f",_maxspeed_mm_s);
        return -1;
    }
    maxSpeed_mm_s = _maxspeed_mm_s;
    return 0;  
}

int TechnoSoftLowDriver::setAcceleration(const double& _acceleration_mm_s2){
    //printf("acceleration = %f, max acceleration = %f", _acceleration,maxAcceleration);
    DPRINT("Chiamata setAcceleration");
    if(_acceleration_mm_s2<=0 || _acceleration_mm_s2>maxAcceleration_mm_s2){
        DERR("Acceleration = %f",_acceleration_mm_s2);
        return -1;
    }
    acceleration_mm_s2 = _acceleration_mm_s2;
    return 0;
}

int TechnoSoftLowDriver::setMaxAcceleration(const double& _maxacceleration_mm_s2){
    
    if(_maxacceleration_mm_s2<=0 || _maxacceleration_mm_s2<acceleration_mm_s2){
        DERR("Max acceleration = %f",_maxacceleration_mm_s2);
        return -1;
    }
    maxAcceleration_mm_s2 = _maxacceleration_mm_s2;
    return 0;  
}

int TechnoSoftLowDriver::setAdditive(const BOOL& _isAdditive){
    DPRINT("Chiamata setAdditive");
    if((_isAdditive!=TRUE) && (_isAdditive!=FALSE)){
        DERR("setAdditive= %d",_isAdditive);
        return -1;
    }   
    isAdditive = _isAdditive;
    return 0;
}

int TechnoSoftLowDriver::setMovement(const short& _movement){
    DPRINT("Chiamata setMovement");
    if((_movement!=UPDATE_NONE) && (_movement!=UPDATE_IMMEDIATE) && (_movement!=UPDATE_ON_EVENT)){
        DERR("setMovement = %d",_movement);
        return -1;
    }
    movement = _movement;   
    return 0;
}

int TechnoSoftLowDriver::setReferenceBase(const short& _referenceBase){
    DPRINT("Chiamata setReferenceBase");
    if((_referenceBase!=FROM_MEASURE) && (_referenceBase!=FROM_REFERENCE)){
        DERR("_referenceBase = %d",_referenceBase);
        return -1;
    }
    referenceBase=_referenceBase; 
    return 0;
}
  
// Set homing parameters
int TechnoSoftLowDriver::sethighSpeedHoming(const double& _highSpeedHoming_mm_s){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    DPRINT("Chiamata sethighSpeedHoming");
    if(_highSpeedHoming_mm_s<=0 || _highSpeedHoming_mm_s>maxHighSpeedHoming_mm_s){
        return -1;
    }
    highSpeedHoming_mm_s = -_highSpeedHoming_mm_s;
    return 0;
}

int TechnoSoftLowDriver::setMaxhighSpeedHoming(const double& _maxhighSpeedHoming_mm_s){
    
     if(_maxhighSpeedHoming_mm_s<=0 || _maxhighSpeedHoming_mm_s<-highSpeedHoming_mm_s || _maxhighSpeedHoming_mm_s<maxLowSpeedHoming_mm_s){
        return -1;
    }
    maxHighSpeedHoming_mm_s = _maxhighSpeedHoming_mm_s;
    return 0;
}

int TechnoSoftLowDriver::setlowSpeedHoming(const double& _lowSpeedHoming_mm_s){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    DPRINT("Chiamata setlowSpeedHoming");
    if(_lowSpeedHoming_mm_s<=0 || _lowSpeedHoming_mm_s>maxLowSpeedHoming_mm_s){
        return -1;
    }
    lowSpeedHoming_mm_s = _lowSpeedHoming_mm_s;
    return 0;
}

int TechnoSoftLowDriver::setMaxlowSpeedHoming(const double& _maxlowSpeedHoming_mm_s){
    
    if(_maxlowSpeedHoming_mm_s<=0 || _maxlowSpeedHoming_mm_s<lowSpeedHoming_mm_s || _maxlowSpeedHoming_mm_s>maxHighSpeedHoming_mm_s){
        return -1;
    }
    maxLowSpeedHoming_mm_s = _maxlowSpeedHoming_mm_s;
    return 0;
}

int TechnoSoftLowDriver::setaccelerationHoming(const double&  _accelerationHoming_mm_s2){
    //printf("acceleration = %f, max acceleration = %f", _acceleration,maxAcceleration);
    DPRINT("Chiamata setaccelerationHoming");
    if(_accelerationHoming_mm_s2<=0 || _accelerationHoming_mm_s2>maxAccelerationHoming_mm_s2){
        return -1;
    }
    accelerationHoming_mm_s2 = _accelerationHoming_mm_s2;
    return 0;
}

int TechnoSoftLowDriver::setMaxAccelerationHoming(const double&  _maxAccelerationHoming_mm_s2){
    //printf("acceleration = %f, max acceleration = %f", _acceleration,maxAcceleration);
    if(_maxAccelerationHoming_mm_s2<=0 || _maxAccelerationHoming_mm_s2<accelerationHoming_mm_s2){
        return -1;
    }
    maxAccelerationHoming_mm_s2 = _maxAccelerationHoming_mm_s2;
    return 0;
}

int TechnoSoftLowDriver::setAdditiveHoming(const BOOL& _isAdditiveHoming){
    DPRINT("Chiamata setAdditiveHoming");
    if(_isAdditiveHoming!=TRUE && _isAdditiveHoming!=FALSE){
        return -1;
    }   
    isAdditiveHoming = _isAdditiveHoming;
    return 0;
}

int TechnoSoftLowDriver::setMovementHoming(const short& _movementHoming){
    DPRINT("Chiamata setMovementHoming");
    if((_movementHoming!=UPDATE_NONE) && (_movementHoming!=UPDATE_IMMEDIATE) && (_movementHoming!=UPDATE_ON_EVENT)){
        return -1;
    }
    movementHoming = _movementHoming;   
    return 0;
}

int TechnoSoftLowDriver::setReferenceBaseHoming(const short& _referenceBaseHoming){
    DPRINT("Chiamata setReferenceBaseHoming");
    if((_referenceBaseHoming!=FROM_MEASURE) && (_referenceBaseHoming!=FROM_REFERENCE)){
        return -1;
    }
    referenceBaseHoming=_referenceBaseHoming; 
    return 0;
}

// Set encoder lines
int TechnoSoftLowDriver::setEncoderLines(double& _encoderLines){
    DPRINT("Chiamata setEncoderLines");
    if(_encoderLines<=0){
        return -1;
    }   
    n_encoder_lines=_encoderLines;
    return 0;
}

int TechnoSoftLowDriver::setConst_mult_technsoft(double& _const_mult_technsoft){
    DPRINT("Chiamata setConst_mult_technsoft");
    if(_const_mult_technsoft<=0){
        return -1;
    }   
    const_mult_technsoft=_const_mult_technsoft;
    return 0;
}

int TechnoSoftLowDriver::setSteps_per_rounds(double& _steps_per_rounds){
    DPRINT("Chiamata setSteps_per_rounds");
    if(_steps_per_rounds<=0){
        return -1;
    }   
    steps_per_rounds=_steps_per_rounds;
    return 0;
}

int TechnoSoftLowDriver::setN_rounds(double& _n_rounds){
    DPRINT("Chiamata setN_rounds");
    if(_n_rounds<=0){
        return -1;
    }   
    n_rounds=_n_rounds;
    return 0;
}
    
int TechnoSoftLowDriver::setLinear_movement_per_n_rounds(double& _linear_movement_per_n_rounds){
    DPRINT("Chiamata setLinear_movement_per_n_rounds");
    if(_linear_movement_per_n_rounds<=0){
        return -1;
    }   
    linear_movement_per_n_rounds=_linear_movement_per_n_rounds;
    return 0;
}  

int TechnoSoftLowDriver::moveAbsoluteSteps(const long& absPosition) const{
    
//    if(!TS_SelectAxis(axisID)){
//        DERR("error selecting axis");
//        return -1;
//    }
    //deltaPosition*=CONST_MULT_TECHNOFT;
    //printf("%ld",deltaPosition);
    //DPRINT("moving axis: %d deltapos %d speed=%f acceleration %f isadditive %d movement %d referencebase %d",axisID,deltaPosition,speed,acceleration,isAdditive,movement,referenceBase);
//    if(speed<0 || speed>MAX_SPEED){
//        return -1;
//    }
//    if(acceleration<0 || acceleration>MAX_ACCELERATION){
//        return -2;
//    }
//    // nota: MAX_SPEED, MAX_ACCELERATION in TechnoSoftLowDriver.h 
//        
//    if((movement!=UPDATE_NONE) || (movement!=UPDATE_IMMEDIATE) || (movement!=UPDATE_ON_EVENT)){
//        return -3;
//    }
//    // nota: UPDATE_NONE, UPDATE_IMMEDIATE, UPDATE_ON_EVENT costanti definite in TML_LIB.h 
//    
//    if((referenceBase!=FROM_MEASURE) || referenceBase!=FROM_REFERENCE){
//        return -4;
//    }
    DPRINT("moving Absolute steps. Axis: %d, absPosition %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,absPosition,speed_mm_s,acceleration_mm_s2,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveAbsolute(absPosition, speed_mm_s, acceleration_mm_s2, movement, referenceBase)){
//        DERR("error absolute step moving");
//        return -2;
//    }
    int random_variable = std::rand();
    if(random_variable<(RAND_MAX/20)) 
        return -1;
    
    return 0;
}

int TechnoSoftLowDriver::getHighSpeedHoming(double& _highSpeedHoming_mm_s){
    
    DPRINT("Valore letto dell'high speed homing %f:", highSpeedHoming_mm_s);
    _highSpeedHoming_mm_s = highSpeedHoming_mm_s;
    return 0;
}

int TechnoSoftLowDriver::moveVelocityHoming(){
    
    //double highSpeedHoming_MicroSteps_s = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*highSpeedHoming_mm_s)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    //double accelerationHoming_MicroSteps_s = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*accelerationHoming_mm_s2/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    DPRINT("(homing) moving velocity. Axis : %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,highSpeedHoming_mm_s,accelerationHoming_mm_s2,movementHoming,referenceBaseHoming);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_MoveVelocity(highSpeedHoming_mm_s, accelerationHoming_mm_s2, movementHoming, referenceBaseHoming)){
        DERR("(homing) Error moving velocity ");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::moveAbsoluteStepsHoming(const long& absPosition) const{
    
    DPRINT("(homing) moving absolute steps. Axis: %d, absPosition %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,absPosition,speed_mm_s,acceleration_mm_s2,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_MoveAbsolute(absPosition, lowSpeedHoming_mm_s, accelerationHoming_mm_s2, movementHoming, referenceBaseHoming)){
        DERR("(homing) Error absolute steps moving");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::stopMotion(){
    
//    if(!TS_SelectAxis(axisID)){
//        return -1;
//    }
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_Stop()){
        return -1;
    }
    DPRINT("Motor with axis = %d is stopped, %s",axisID, TS_GetLastErrorText());
    return 0;
}

int TechnoSoftLowDriver::providePower(){
    DPRINT("provide power to axis:%d",axisID);
//      if(!TS_SelectAxis(axisID)){
//        ERR("ALEDEBUG Error selecting axis");
//        return -1;
//    }
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_Power(POWER_ON)){
        //ERR("ALEDEBUG Error selecting axis");
        return -2;
    }
				
    /*	Wait for power stage to be enabled */
    WORD sAxiOn_flag = 0;
    while(sAxiOn_flag == 0){
        /* Check the status of the power stage */
        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
	    
            //ERR("ALEDEBUG Error TS_ReadStatus");
            return -3;
        }

        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
    }
    //DPRINT("ALEDEBUG correctly powered on");
    //poweron=true;
    return 0;
}

int TechnoSoftLowDriver::stopPower(){
    //DPRINT("stopping power to axis:%d",axisID);

//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_Power(POWER_OFF)){
        return -1;
    }
    DPRINT("Motor with axis id = %d is power off",axisID);
    
    return 0;
}

int TechnoSoftLowDriver::deinit(){ // Identical to TechnoSoftLowDriver::stopPower()
    
    DPRINT("TechnoSoftLowDriver %d object is deallocated",axisID);
    return 0;
}

//int TechnoSoftLowDriver::getCounter(long& tposition){
//    
//    DPRINT("Reading COUNTER position");
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -1;
////    }
//    if(!TS_GetLongVariable("TPOS", tposition)){
//        return -2;
//    }
//    return 0;
//}

int TechnoSoftLowDriver::getCounter(double* deltaPosition_mm){
    
    DPRINT("Reading COUNTER position");
    long tposition;
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_GetLongVariable("TPOS", tposition)){
        return -1;
    }
    *deltaPosition_mm = (tposition*linear_movement_per_n_rounds)/(steps_per_rounds*const_mult_technsoft*n_rounds);
    return 0;
}


//int TechnoSoftLowDriver::getEncoder(long& aposition){
//    
//    DPRINT("Reading ENCODER position");
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -1;
////    }
//    if(!TS_GetLongVariable("APOS", aposition)){
//        return -2;
//    }
//    return 0;
//}



int TechnoSoftLowDriver::getEncoder(double* deltaPosition_mm){
    
    DPRINT("Reading ENCODER position");
    long aposition;
    if(!TS_GetLongVariable("APOS", aposition)){
        return -1;
    }
    *deltaPosition_mm = (aposition*linear_movement_per_n_rounds)/(n_encoder_lines*n_rounds);
    return 0;
}

int TechnoSoftLowDriver::getLVariable(std::string& nameVar, long& var) {
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_GetLongVariable(nameVar.c_str(), var)){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::resetCounter(){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_Execute("SAP 0")){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::resetEncoder(){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_Execute("APOS=0")){
        return -1;
    }
    return 0;
}

//int TechnoSoftLowDriver::getPower(BOOL& powered){
//    
//    powered = FALSE;
//    WORD power;
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_ReadStatus(REG_SRL,power)){
//        return -2;
//    }
//    power = ((power & 1<<15)==0 ? 1 : 0); //La forma generale dell'istruzione di SHIFT A SINISTRA e' del tipo:
//    //variabile_intera << numero_posizioni
//    if (power==1) {
//        powered = FALSE;
//        return -3;
//    }
//    else{
//        powered = TRUE;
//        return -4;
//    }
//}

//int TechnoSoftLowDriver::setFixedVariable(LPCSTR pszName, double value){
////The function converts the value to type fixed and writes it in the TML data
////pszName on the active axis
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_SetFixedVariable(pszName, value)){
//        return -2;
//    }
//    return 0;
//}

//int TechnoSoftLowDriver::abortNativeOperation(){
//    //The function aborts the execution of a TML function launched with a 
//    //cancelable call
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_ABORT()){
//        return -2;
//    }
//    return 0;
//}


//int TechnoSoftLowDriver::executeTMLfunction(std::string& pszFunctionName){
//    // The function commands the active axis to execute the TML function stored
//    //at pszFunctionName
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_CancelableCALL_Label(pszFunctionName.c_str())){
//        printf("executeTMLfunction: %s\n",TS_GetLastErrorText());
//        return -2;
//    }
//    return 0;
//}

//int TechnoSoftLowDriver::setDecelerationParam(double deceleration){
//    // 
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_QuickStopDecelerationRate(deceleration)){
//        return -2;
//    }
//    return 0;
//}

//int TechnoSoftLowDriver::setVariable(LPCSTR pszName, long value){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_SetLongVariable(pszName, value)){
//	return -2;
//    }
//    return 0;
//}

//int TechnoSoftLowDriver::readHomingCallReg(short selIndex, WORD& status){
//    
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_ReadStatus(selIndex, status))
//        return FALSE;
//
//    status=((status & 1<<8) == 0 ? 1 : 0);
//    return 0;
//}

int TechnoSoftLowDriver::setEventOnLimitSwitch(short lswType, short transitionType, BOOL waitEvent, BOOL enableStop){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_SetEventOnLimitSwitch(lswType, transitionType, waitEvent, enableStop)){
	return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::setEventOnMotionComplete(BOOL waitEvent, BOOL enableStop){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_SetEventOnMotionComplete(waitEvent,enableStop)){ 
	return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::checkEvent(BOOL& event){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_CheckEvent(event)){ 
        return -2;
    }
    return 0;
}

//int TechnoSoftLowDriver::setPosition(const long& posValue){
////  if(!TS_SelectAxis(axisID)){
////        return -1;
////  }
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_SetPosition(posValue)){ 
//        DPRINT("Error at setting position: %s",TS_GetLastErrorText());
//        return -1;
//    }
//    return 0;
//}

int TechnoSoftLowDriver::getStatusOrErrorReg(const short& regIndex, WORD& contentRegister, std::string& descrErr){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_ReadStatus(regIndex,contentRegister)){

        //DERR("Error at the register reading: %s",TS_GetLastErrorText());
        //descrErr.assign(TS_GetLastErrorText());
        descrErr=descrErr+" Error reading status: "+TS_GetLastErrorText();
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::resetFault(){
    
    if(!TS_ResetFault()){
         return -2;
         // Note: the drive-motor will return to FAULT status (SRH.15=1) if there are
         // errors when the function is executed)
    }
    return 0;
}

int TechnoSoftLowDriver::selectAxis(){
     
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    int random_variable = std::rand();
    if(random_variable<(RAND_MAX/20)) 
        return -1;
    
    return 0;
}

// retrieve firmware version of the active drive
int TechnoSoftLowDriver::getFirmwareVers(char* firmwareVers){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!MSK_GetDriveVersion(firmwareVers)){
        DERR("Errore lettura versione driver");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::getinternalHomingStateDefault(){
    
    return internalHomingStateDefault;
}

int TechnoSoftLowDriver::getinternalHomingStateHoming2(){
    
    return internalHomingStateHoming2;
}

int getHoming2Procedure();

//int TechnoSoftLowDriver::resetSetup(){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
////    if(!TS_Execute("ENDINIT")){
////        //descrErr=descrErr+" "+TS_GetLastErrorText()+". ";
////        DERR("Failed TS_Execute command");
////        return -17;
////    }
//    
//    if(!TS_Save()){
//         DERR("failed Low driver saving current setup: %s",TS_GetLastErrorText());
//         return -1; 
//    }
////    char szDriveVersion[100];
////    //char* szDriveVersion;
////    if(!MSK_GetDriveVersion(szDriveVersion))
////        DERR("Errore lettura versione driver");
////        
////    DPRINT("%s",szDriveVersion);
//    
//    if(!TS_Reset()){
//        DERR("failed Low driver reset: %s",TS_GetLastErrorText());
//        return -1; 
//    } 
//    
//    sleep(10);
//    
//    if(!MSK_SetBaudRate(9600)){
//        DERR("MSK_SetBaudRate 1 %s",TS_GetLastErrorText());
//        //return -2;
//        return -2;
//    }
//    sleep(3);
//    
//    if(!MSK_SetBaudRate(115200)){
//        DERR("MSK_SetBaudRate 2 %s",TS_GetLastErrorText());
//        //return -2;
//        return -2;
//    }
//    
//    sleep(3);
//
////    if(!TS_SelectAxis(axisID)){
////       DERR("failed to select axis %d",axisID);
////       return -4;
////    }
////    
//    if(!TS_DriveInitialisation()){
//        DERR("failed Low driver initialisation %s",TS_GetLastErrorText());
//        return -4;
//    }
////    
//    if(providePower()<0){ // che in teoria non ci andrebbe qui dentro...
//        DERR("failed power providing %s,", TS_GetLastErrorText());// failed power providing Send message timeout.
//        return -5;
//    }
//    
////    if((my_channel->open()<0)){
////         DERR("error opening channel");
////         return -2;
////    }
//     /*	Execute the initialization of the drive (ENDINIT) */
//    
////    sleep(2);
////    
////    if(!TS_SetupAxis(axisID, axisRef)){
////        DERR("failed to setup axis %d",axisID);
////        return -15;
////    }
////    
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -4;
////    }
//    
//    // SET THE SERIAL COMMUNICATION BAUD RATE TO 9600. AFTER RESET THE DRIVE COMMUNICATES USING 9600 BPS
//    //....................
//    //....................
//    //....................
//    
//    // RESTORE THE SERIAL COMMUNICATIONE BAUD RATE
//    //....................
//    //....................
//    //....................
//    
//    // Settare il registro per la lettura dell'encoder
////    if(!TS_Execute("SCR=0x4338")){
////        //descrErr=descrErr+" "+TS_GetLastErrorText()+". ";
////        DERR("Failed TS_Execute command");
////        return -17;
////    }
////    
////    if(!TS_DriveInitialisation()){
////        DERR("failed Low driver initialisation: %s",TS_GetLastErrorText());
////        return -5;
////    }
//    
//    return -6;
//}

/*****************************************************************/
/*****************************************************************/
 void SerialCommChannelTechnosoft::PrintChannel()
{
  DPRINT("%d %s %d %d\n",this->fd, this->pszDevName.c_str(),this->btType,this->baudrate);
}
/*****************************************************************/
 int  SerialCommChannelTechnosoft::getFD() {return this->fd;}
 
 

 
 
 
 