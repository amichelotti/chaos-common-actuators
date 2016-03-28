
#include "TechnoSoftLowDriver.h"
#include <common/debug/core/debug.h>
#include <iostream>

using namespace common::actuators::models;

//--------------------------------------------

void ElectricPowerException::badElectricPowerInfo(){
    
    std::cerr<< "The electrical power has not been turned off." << std::endl; 
}


SerialCommChannelTechnosoft::SerialCommChannelTechnosoft(const std::string& pszDevName,const BYTE btType,const DWORD baudrate){
    init(pszDevName,btType,baudrate);
}
std::string  SerialCommChannelTechnosoft::getDevName(){return this->pszDevName;}
int SerialCommChannelTechnosoft::getbtType(){return this->btType;}
int SerialCommChannelTechnosoft::getbaudrate(){return this->baudrate;}
int SerialCommChannelTechnosoft::init(const std::string& _pszDevName,const BYTE& _btType,const DWORD& _baudrate){
    DPRINT("initializing dev %s type %d baud %d",_pszDevName.c_str(),_btType,_baudrate);
    pszDevName=_pszDevName;
    btType = _btType;
    baudrate = _baudrate;
    fd = -1;
    return 0;
}

void SerialCommChannelTechnosoft::close(){

    if(fd!=-1){
	TS_CloseChannel(fd); // chiusura canale di comunicazione
    }
}

SerialCommChannelTechnosoft::~SerialCommChannelTechnosoft(){
    close();
}

int SerialCommChannelTechnosoft::open(int hostID){
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

TechnoSoftLowDriver::channel_map_t TechnoSoftLowDriver::channels;


//----------------------------------------------
TechnoSoftLowDriver::TechnoSoftLowDriver(const std::string devName,const std::string name){
    alreadyopenedChannel = false;
    poweron = false;
    channel_map_t::iterator i=channels.find(devName);
    if(i!=channels.end()){
        my_channel = i->second;
        // In questo caso non dovrò più provare ad aprire il canale di comunicazione
        // nella sucessiva procedura di inizializzazione  del canale + drive/motor
        // Settiamo dunque a true questo stato
        alreadyopenedChannel = true;
        // Verrà sfruttato il canale di comunicazione desiderato che è correntemente aperto
    } else {
        my_channel = channel_psh(new SerialCommChannelTechnosoft(devName));
        //*****Nota il nuovo canale creato deve essere inserito nella mappa:
        //channels.insert( std::pair<std::string,channel_psh>(devName,my_channel)); // IPOTESI TEMPORANEA: QUESTA FUNZIONE NON GENERA MAI ECCEZIONE 
    }
    DPRINT("created channel  dev %s name %s", devName.c_str(),name.c_str());
}

TechnoSoftLowDriver::~TechnoSoftLowDriver(){
    deinit();
}

int TechnoSoftLowDriver::init(const std::string& setupFilePath,const int& axisID, const double speed, const double acceleration, const BOOL isAdditive, const short moveMoment, const short referenceBase, const int encoderLines)
{
    int resp=0;
    
    /*	Load the *.t.zip with setup data generated with EasyMotion Studio or EasySetUp, for axisID*/
    int axisRef;
    DPRINT("during low driver init"); 
    
    if(!alreadyopenedChannel){
        if((my_channel->open()<0)){
            DERR("error opening channel");
            resp = -1;
        }
    }
    DPRINT("channel opened");
    
    // Indipendentemente dal fatto che il canale era già stato aperto oppure 
    // che ne sia stato appena aperto un nuovo, segue la inizializzazione del drive/motor
    // attraverso (l'ULTIMO) canale che è stato aperto
    axisRef = TS_LoadSetup(setupFilePath.c_str());
    if(axisRef < 0){
        DERR("LoadSetup failed \"%s\"",setupFilePath.c_str());
        resp = -2;
    }
    
    /*	Setup the axis based on the setup data previously, for axisID*/
    if(!TS_SetupAxis(axisID, axisRef)){
        DERR("failed to setup axis %d",axisID);
        resp = -3;
    }
    
    if(!TS_SelectAxis(axisID)){
        DERR("failed to select axis %d",axisID);
        resp = -4;
    }

    /*	Execute the initialization of the drive (ENDINIT) */
    if(!TS_DriveInitialisation()){
        DERR("failed Low driver initialisation");
        resp = -5;
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
    //printf("esito providePower: %d\n",providePower());
    
    DPRINT("ALEDEBUG Caschera!!! Exiting init");
    
    // DA TOGLIERE IL PRIMA POSSIBILE IL SEGUENTE BLOCCO DI CODICE
    if(providePower()<0){
        DERR("failed power providing");
        resp = -6;
    }
    
    if(resp==0 && !alreadyopenedChannel){
        channels.insert(std::pair<std::string,channel_psh>(devName,my_channel)); 
        // IPOTESI TEMPORANEA: QUESTA FUNZIONE NON può GENERAre MAI ECCEZIONE
    }
    return resp;
}

channel_psh getMyChannel(){
    
    return my_channel;
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
        ERR("ALEDEBUG Error selecting axis");
        return -1;
    }

    if(!TS_Power(POWER_ON)){
        ERR("ALEDEBUG Error selecting axis");
        return -2;
    }
				
    /*	Wait for power stage to be enabled */
    WORD sAxiOn_flag = 0;
    while(sAxiOn_flag == 0){
        /* Check the status of the power stage */
        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
	    
            ERR("ALEDEBUG Error TS_ReadStatus");
            return -3;
        }

        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
    }
    DPRINT("ALEDEBUG correctly powered on");
    poweron=true;
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
    
    // DA TOGLIERE IL PRIMA POSSIBILE QUESTA ISTRUZIONE
    if(poweron){
        if(stopPower()<0){ // questa istruzione potrebbe restituire errore se il canale non è stato aperto
                     // oppure se il drive/motor non è stato inizializzato correttamente, 
                     // oppure se l'azione di erogazione dell'alimentazione non ha avuto buon fine.
                     // Errore che comunque non compromette il corretto svolgimento del programma.
            // ATTENZIONE: DOVRA ESSERE GENERATA UNA ECCEZIONE IN CASO L'OPERAZIONE DI 
            // SPEGNIMENTO DELL'ALIMENTAZIONE DEL MOTORE NON È ANDATA A BUON FINE
            // ..................
            // ..................
            // ..................
            throw ElectricPowerException();
        }
    // COSI COME BISOGNA TOGLIERE IL PRIMA POSSIBILE L'ISTRUZIONE providePower
    // in INIT
    }
    
    if(my_channel!=NULL){
        if(my_channel.use_count()==2){
            // In questo caso bisogna eliminare anche la riga relativa all'oggetto canale utilizzato 
            // ,unicamente utilizzato dall'oggetto this, nella mappa statica.
            // Questo garantisce che una volta distrutto l'oggetto this, anche l'oggetto canale
            // sarà automaticamente deallocato (e quindi chiuso anche il canale di comunicazione)
            channels.erase(devName);
        }
            
        my_channel.reset(); // Setto a NULL lo smart pointer (shared),
                            // cosicché se era l'unico a puntare all'oggetto canale,
                            // l'oggetto canale sarà anche esso deallocato (ed in quel caso
                            // verrà anche chiuso l'associatocanale di comunicazione )
    }
    return 0;
}

int TechnoSoftLowDriver::getCounter(long& tposition){
   // DPRINT("getting counter");
    DPRINT("Reading COUNTER position");
    if(!TS_SelectAxis(axisID)){
        return -1;
    }

    if(!TS_GetLongVariable("TPOS", tposition)){
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::getEncoder(long& aposition){
    DPRINT("Reading ENCODER position");
    if(!TS_SelectAxis(axisID)){
        return -1;
    }

   
    if(!TS_GetLongVariable("APOS", aposition)){
        return -2;
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


int TechnoSoftLowDriver::executeTMLfunction(std::string& pszFunctionName){
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
  if(!TS_SelectAxis(axisID)){
        return -1;
  }
  
    if(!TS_SetPosition(posValue)) 
        return -1;
    return 0;
}

int TechnoSoftLowDriver::getStatusOrErrorReg(short& regIndex, WORD& contentRegister, std::string& descrErr){

  if (my_channel==NULL)
  {
	DPRINT("mychannel == null");
  }
  else
  {
	//DPRINT("my_channel not null ");
	my_channel->PrintChannel();
	//int a=my_channel->close();
	//my_channel->init(my_channel->getDevName(),my_channel->getbtType(),my_channel->getbaudrate());
	//DPRINT("ALEDEBUG after second init ");
	//TS_SelectChannel(my_channel->getFD());
  }
  int count=10;
  while ( (count >0) && (!TS_SelectAxis(axisID)))
  {
    ERR("%d cannot select axis %d (%s)",count,axisID,TS_GetLastErrorText());
    count--;
    my_channel->close();
	if (!TS_OpenChannel(my_channel->getDevName().c_str(),(BYTE)my_channel->getbtType(),HOST_ID, my_channel->getbaudrate())) {
	  DPRINT("ALEDEBUG failed open channel");
         }
	 else
	 {
	   DPRINT("ALEDEBUG opened channel");
	   my_channel->PrintChannel();
	   TS_SelectChannel(my_channel->getFD());
	 }
  }
  if (count == 0)
  {
	ERR("ALEDEBUG failed to select axis");
	return -1;
  }
 

    DPRINT("Reading status at %d",regIndex);
    descrErr.assign("");
    if(!TS_ReadStatus(regIndex,contentRegister)){
	
        ERR("Error at the register reading: %s",TS_GetLastErrorText());
        descrErr.assign(TS_GetLastErrorText());
        return -1;
    }
    return 0;
}
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
 void SerialCommChannelTechnosoft::PrintChannel()
{
  DPRINT("%d %s %d %d\n",this->fd, this->pszDevName.c_str(),this->btType,this->baudrate);
}
/*****************************************************************/
 int  SerialCommChannelTechnosoft::getFD() {return this->fd;}
