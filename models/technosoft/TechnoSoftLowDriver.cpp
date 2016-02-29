
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
    return 0;
}

SerialCommChannelTechnosoft::~SerialCommChannelTechnosoft(){
    this->close();
}

void SerialCommChannelTechnosoft::close(){

    if(this->fd!=-1){
	TS_CloseChannel(this->fd);
    }
}

int SerialCommChannelTechnosoft::open(int hostID){
    int resp;
    /*	Open the comunication channel: COM1, RS232, 1, 115200 */
   DPRINT("opening dev %s type %d baud %d, hostid:%d",pszDevName.c_str(),btType,baudrate,hostID);
   resp=TS_OpenChannel(pszDevName.c_str(), btType, hostID, baudrate);
    if(resp < 0){
        DERR("failed opening channel");
        return -1;
    }
    this->fd = resp;
    DPRINT("Openchannel resp=%d",resp);
    return 0;
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
        return -1;
    }
    axisRef = TS_LoadSetup(setupFilePath.c_str());
    if(axisRef < 0){
        DERR("LoadSetup failed \"%s\"",setupFilePath.c_str());
        return -2;
    }
    
    /*	Setup the axis based on the setup data previously, for axisID*/
    if(!TS_SetupAxis(axisID, axisRef)){
        DERR("failed to setup axis %d",axisID);
        return -3;
    }
    
    if(!TS_SelectAxis(axisID)){
      DERR("failed to select axis %d",axisID);

        return -4;
    }

    /*	Execute the initialization of the drive (ENDINIT) */
    if(!TS_DriveInitialisation()){
        DERR("Low driver initialisation");
        return -5;
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

int TechnoSoftLowDriver::moveRelativeSteps(const long& deltaPosition){
      if(!TS_SelectAxis(axisID)){
          DERR("error selecting axis");
        return -1;
    }

    //deltaPosition*=CONST_MULT_TECHNOFT;
    //printf("%ld",deltaPosition);
    DPRINT("moving axis: %d deltapos %d speed=%f acceleration %f isadditive %d movement %d referencebase %d",axisID,deltaPosition,speed,acceleration,isAdditive,movement,referenceBase);
    if(!TS_MoveRelative(deltaPosition, speed, acceleration, isAdditive, movement, referenceBase)){
        DERR("error relative moving");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::moveAbsoluteSteps(const long& absPosition){
    
    if(!TS_SelectAxis(axisID)){
        DERR("error selecting axis");
        return -1;
    }
    //deltaPosition*=CONST_MULT_TECHNOFT;
    //printf("%ld",deltaPosition);
    //DPRINT("moving axis: %d deltapos %d speed=%f acceleration %f isadditive %d movement %d referencebase %d",axisID,deltaPosition,speed,acceleration,isAdditive,movement,referenceBase);
    if(!TS_MoveAbsolute(absPosition, this->speed, this->acceleration, movement, referenceBase)){
        DERR("error absolute moving");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::stopMotion(){
    DPRINT("stop axis:%d",axisID);
    if(!TS_SelectAxis(axisID)){
        return -1;
    }

    if(!TS_Stop()){
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::providePower(){
    DPRINT("provide power to axis:%d",axisID);
      if(!TS_SelectAxis(axisID)){
        return -1;
    }

    if(!TS_Power(POWER_ON)){
        return -2;
    }
				
    /*	Wait for power stage to be enabled */
    WORD sAxiOn_flag = 0;
    while(sAxiOn_flag == 0){
        /* Check the status of the power stage */
        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
            return -3;
        }
        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
    }
    return 0;
}

int TechnoSoftLowDriver::stopPower(){
    DPRINT("stop power to axis:%d",axisID);

    if(!TS_SelectAxis(axisID)){
        return -1;
    }

    if(!TS_Power(POWER_OFF)){
        return -2;
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

int TechnoSoftLowDriver::getCounter(long& tposition){
   // DPRINT("getting counter");
    if(!TS_SelectAxis(axisID)){
        return -1;
    }

    if(!TS_GetLongVariable("TPOS", tposition)){
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::getEncoder(long& aposition){
   
    if(!TS_GetLongVariable("APOS", aposition)){
        return -1;
    }
    return 0;
}

int getLVariable(std::string& nameVar, long& var){
    if(!TS_GetLongVariable(nameVar.c_str(), var)){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::resetCounter(){
    
    if(!TS_Execute("SAP 0")){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::resetEncoder(){
    
    if(!TS_Execute("APOS=0")){
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::getPower(BOOL& powered){
    
    powered = FALSE;
    WORD power;
    if(!TS_ReadStatus(REG_SRL,power)){
        return -1;
    }
    power = ((power & 1<<15)==0 ? 1 : 0); //La forma generale dell'istruzione di SHIFT A SINISTRA e' del tipo:
    //variabile_intera << numero_posizioni
    if (power==1) {
        powered = FALSE;
        return -2;
    }
    else{
        powered = TRUE;
        return -3;
    }
}

int TechnoSoftLowDriver::setFixedVariable(LPCSTR pszName, double value){
//The function converts the value to type fixed and writes it in the TML data
//pszName on the active axis
    if(!TS_SetFixedVariable(pszName, value)) 
        return -1;
    return 0;
}

int TechnoSoftLowDriver::abortNativeOperation(){
    //The function aborts the execution of a TML function launched with a 
    //cancelable call
    if(!TS_ABORT()) 
        return -1;
    return 0;
}


BOOL TechnoSoftLowDriver::executeTMLfunction(std::string& pszFunctionName){
    // The function commands the active axis to execute the TML function stored
    //at pszFunctionName
    if(!TS_CancelableCALL_Label(pszFunctionName.c_str()))
        return FALSE;
    return TRUE;

}

int TechnoSoftLowDriver::setDecelerationParam(double deceleration){
    // 
    if(!TS_QuickStopDecelerationRate(deceleration))
        return -1;
    return 0;
}

int TechnoSoftLowDriver::setVariable(LPCSTR pszName, long value){
    if(!TS_SetLongVariable(pszName, value)) 
	return -1;
    return 0;
}

int TechnoSoftLowDriver::readHomingCallReg(short selIndex, WORD& status){
    
    if(!TS_ReadStatus(selIndex, status))
        return FALSE;

    status=((status & 1<<8) == 0 ? 1 : 0);
    return 0;
}

int TechnoSoftLowDriver::setEventOnLimitSwitch(short lswType = LSW_NEGATIVE , short transitionType = TRANSITION_HIGH_TO_LOW , BOOL waitEvent = 1, BOOL enableStop = 0){
    
    if(!TS_SetEventOnLimitSwitch(lswType, transitionType, waitEvent, enableStop)) 
	return -1;
    return 0;
}

int TechnoSoftLowDriver::setEventOnMotionComplete(BOOL waitEvent=1, BOOL enableStop=0){
    
    if(!TS_SetEventOnMotionComplete(waitEvent,enableStop)) 
	return -1;
    return 0;
}

int TechnoSoftLowDriver::checkEvent(BOOL& event){
    
    if(!TS_CheckEvent(event)) 
        return -1;
    return 0;
}

int TechnoSoftLowDriver::setPosition(const long& posValue){
    
    if(!TS_SetPosition(posValue)) 
        return -1;
    return 0;
}

int TechnoSoftLowDriver::getStatusOrErrorReg(short& regIndex, WORD& contentRegister, std::string& descrErr){
    
    DPRINT("Reading status");
    descrErr.assign("");
    if(!TS_ReadStatus(regIndex,contentRegister)){
	
        DERR("Error at the register reading: %s",TS_GetLastErrorText());
        descrErr.assign(TS_GetLastErrorText());
        return -1;
    }
    return 0;
}
