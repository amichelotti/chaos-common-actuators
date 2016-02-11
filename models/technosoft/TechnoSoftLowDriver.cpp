
#include "TechnoSoftLowDriver.h"
#include <common/debug/core/debug.h>

using namespace common::actuators::models;

//--------------------------------------------
SerialCommChannelTechnosoft::SerialCommChannelTechnosoft(const std::string& pszDevName,const BYTE btType,const DWORD baudrate){
    init(pszDevName,btType,baudrate);
}
int SerialCommChannelTechnosoft::init(const std::string& _pszDevName,const BYTE& btType,const DWORD& baudrate){
    DPRINT("initializing dev %s type %d baud %d",_pszDevName.c_str(),btType,baudrate);
    pszDevName=_pszDevName;
    this->btType=btType;
    this->baudrate = baudrate;
    this->fd = -1;
}

SerialCommChannelTechnosoft::~SerialCommChannelTechnosoft(){
    this->close();
}

void SerialCommChannelTechnosoft::close(){

	if(this->fd!=-1){
		TS_CloseChannel(this->fd);
	}
}

BOOL SerialCommChannelTechnosoft::open(int hostID){
    int resp;
    /*	Open the comunication channel: COM1, RS232, 1, 115200 */
   DPRINT("opening dev %s type %d baud %d, hostid:%d",pszDevName.c_str(),btType,baudrate,hostID);
   resp=TS_OpenChannel(pszDevName.c_str(), btType, hostID, baudrate);
    if(resp < 0){
        DERR("failed opening channel");
        return FALSE;
    }
    this->fd = resp;
    DPRINT("Openchannel resp=%d",resp);
    return TRUE;
}

TechnoSoftLowDriver::channel_map_t TechnoSoftLowDriver::channels;


//--------------------------------------------

TechnoSoftLowDriver::TechnoSoftLowDriver(const std::string devName,const std::string name){
    channel_map_t::iterator i=channels.find(devName);
    if(i!=channels.end()){
        my_channel = i->second;
    } else {
        my_channel = channel_psh(new SerialCommChannelTechnosoft(devName));
        
        
    }
}

TechnoSoftLowDriver::~TechnoSoftLowDriver(){
    deinit();
}

int TechnoSoftLowDriver::init(const std::string& setupFilePath,const int& axisID, const double speed, const double acceleration, const BOOL isAdditive, const short moveMoment, const short referenceBase, const int encoderLines){
    
    /*	Load the *.t.zip with setup data generated with EasyMotion Studio or EasySetUp, for axisID*/
    int axisRef;
    if((my_channel==NULL) || (my_channel->open()==FALSE)){
        DERR("error opening channel");
        return -10;
    }
    axisRef = TS_LoadSetup(setupFilePath.c_str());
    if(axisRef < 0){
        DERR("LoadSetup failed \"%s\"",setupFilePath.c_str());
        return -1;
    }
    
    /*	Setup the axis based on the setup data previously, for axisID*/
    if(!TS_SetupAxis(axisID, axisRef)){
        DERR("failed to setup axis %d",axisID);
        return -2;
    }
    
    if(!TS_SelectAxis(axisID)){
      DERR("failed to select axis %d",axisID);

        return -3;
    }

    /*	Execute the initialization of the drive (ENDINIT) */
    if(!TS_DriveInitialisation()){
        DERR("Low driver initialisation");

        return -4;
    }

    // Inizializziamo l'asse ID del motore
    this->axisID=axisID;
    this->axisRef=axisRef;
    // Nota: in realtà l'axisID e l'axisRef potrebbero anche non essere definiti tra gli attributi privati perché non sono parametri dei successivi comandi per la movimentazione o lettura dei parametri
    //this->relPosition=relPosition;
    this->speed=speed;
    this->acceleration=acceleration;
    this->isAdditive=isAdditive;
    this->movement=moveMoment;
    this->referenceBase=referenceBase;
    
    this->encoderLines= encoderLines;
    return providePower();
    
}


BOOL TechnoSoftLowDriver::moveRelativeSteps(const long& deltaPosition){
      if(!TS_SelectAxis(axisID)){
          DERR("error selecting axis");
        return -3;
    }

    //deltaPosition*=CONST_MULT_TECHNOFT;
    //printf("%ld",deltaPosition);
      DPRINT("moving axis: %d deltapos %d speed=%f acceleration %f isadditive %d movement %d referencebase %d",axisID,deltaPosition,speed,acceleration,isAdditive,movement,referenceBase);
    if(!TS_MoveRelative(deltaPosition, this->speed, this->acceleration, this->isAdditive,this->movement,this->referenceBase)){
        DERR("error moving");
        return FALSE;
    }
    return TRUE;
}

BOOL TechnoSoftLowDriver::stopMotion(){
  DPRINT("stop axis:%d",axisID);
    if(!TS_SelectAxis(axisID)){
        return -3;
    }

      if(!TS_Stop()){
          return FALSE;
      }
      return TRUE;
}

int TechnoSoftLowDriver::providePower(){
    DPRINT("provide power to axis:%d",axisID);
      if(!TS_SelectAxis(axisID)){
        return -3;
    }

    if(!TS_Power(POWER_ON)){
        return -1;
    }
				
    /*	Wait for power stage to be enabled */
    WORD sAxiOn_flag = 0;
    while(sAxiOn_flag == 0){
        /* Check the status of the power stage */
        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
            return -2;
        }
        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
    }
    return 0;
}

int TechnoSoftLowDriver::stopPower(){
      DPRINT("stop power to axis:%d",axisID);

      if(!TS_SelectAxis(axisID)){
        return -3;
    }

    if(!TS_Power(POWER_OFF)){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::deinit(){ // Identical to TechnoSoftLowDriver::stopPower()
    DPRINT("deinitializing");
    //if(!TS_Power(POWER_OFF)){
        //return -1;
    //}
    stopPower();
    if(my_channel!=NULL){
        my_channel.reset();
    }
    return 0;
}

BOOL TechnoSoftLowDriver::getCounter(long& tposition){
   // DPRINT("getting counter");
  if(!TS_SelectAxis(axisID)){
        return -3;
    }

    if(!TS_GetLongVariable("TPOS", tposition)){
        return FALSE;
    }
    return TRUE;
}

BOOL TechnoSoftLowDriver::getEncoder(long& aposition){
   
    if(!TS_GetLongVariable("APOS", aposition)){
        return FALSE;
    }
    return TRUE;
}

BOOL TechnoSoftLowDriver::resetCounter(){
    
    if(!TS_Execute("SAP 0")){
        return FALSE;
    }
    return TRUE;
}

BOOL TechnoSoftLowDriver::resetEncoder(){
    
    if(!TS_Execute("APOS=0")){
        return FALSE;
    }
    return TRUE;
}

BOOL TechnoSoftLowDriver::getPower(BOOL& powered){
    
    powered = FALSE;
    WORD power;
    if(!TS_ReadStatus(REG_SRL,power)){
        return FALSE;
    }
    power = ((power & 1<<15)==0 ? 1 : 0); //La forma generale dell'istruzione di SHIFT A SINISTRA e' del tipo:
    //variabile_intera << numero_posizioni
    if (power==1) {
        powered = FALSE;
        return TRUE;
    }
    else{
        powered = TRUE;
        return TRUE;
    }
}
