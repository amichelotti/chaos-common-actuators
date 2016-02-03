
#include "TechnoSoftLowDriver.h"


using namespace common::actuators::technosoft;

//--------------------------------------------
SerialCommChannelTechnosoft::SerialCommChannelTechnosoft(const std::string& pszDevName,const BYTE& btType,const DWORD& baudrate){
    strcpy(this->pszDevName,pszDevName.c_str());
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
    if((resp=TS_OpenChannel(this->pszDevName, this->btType, hostID, this->baudrate)) < 0){
        return FALSE;
    }
    this->fd = resp;
    return TRUE;
}

//--------------------------------------------
TechnoSoftLowDriver::TechnoSoftLowDriver(const std::string& setupFilePath)
{
    strcpy(this->setupFilePath,setupFilePath.c_str());
}

int TechnoSoftLowDriver::init(const int& axisID, const double& speed, const double& acceleration, const BOOL& isAdditive, const short& moveMoment, const short& referenceBase){
    
    /*	Load the *.t.zip with setup data generated with EasyMotion Studio or EasySetUp, for axisID*/
    int axisRef;
    axisRef = TS_LoadSetup(this->setupFilePath);
    if(axisRef < 0){
        return -1;
    }
    
    /*	Setup the axis based on the setup data previously, for axisID*/
    if(!TS_SetupAxis(axisID, axisRef)){
        return -2;
    }
    
    if(!TS_SelectAxis(axisID)){
        return -3;
    }

    /*	Execute the initialization of the drive (ENDINIT) */
    if(!TS_DriveInitialisation()){
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
    
    return 0;
}


BOOL TechnoSoftLowDriver::moveRelativeSteps(const long& deltaPosition){
    
    //deltaPosition*=CONST_MULT_TECHNOFT;
    //printf("%ld",deltaPosition);
    if(!TS_MoveRelative(deltaPosition, this->speed, this->acceleration, this->isAdditive,this->movement,this->referenceBase)){
        return FALSE;
    }
    return TRUE;
}

BOOL TechnoSoftLowDriver::stopMotion(){

      if(!TS_Stop()){
          return FALSE;
      }
      return TRUE;
}

int TechnoSoftLowDriver::providePower(){
    
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
    
    if(!TS_Power(POWER_OFF)){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::deinit(){ // Identical to TechnoSoftLowDriver::stopPower()
    
    //if(!TS_Power(POWER_OFF)){
        //return -1;
    //}
    return 0;
}

BOOL TechnoSoftLowDriver::getCounter(long& tposition){

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
