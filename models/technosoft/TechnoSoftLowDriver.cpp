
#include "TechnoSoftLowDriver.h"
#include <common/debug/core/debug.h>
#include <iostream>

using namespace common::actuators::models;

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

}

TechnoSoftLowDriver::~TechnoSoftLowDriver(){

}    
 

double speedfromMMsToIU(double _speed_mm_s){
    
    return (N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360*_speed_mm_s) / CONVERSION_FACTOR_DEG_UI;
}

double speedfromIUTOMMs(double _speed_IU){
    
    return (_speed_IU*CONVERSION_FACTOR_DEG_UI)/(N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360);
}

double accelerationfromMMs2ToIU(double _acceleration_mm_s2){
    
    return (N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360*_acceleration_mm_s2) / CONVERSION_FACTOR_DEGs2_UI;
}

double accelerationfromIUToMMs2(double _acceleration_IU){
    
    return (_acceleration_IU*CONVERSION_FACTOR_DEGs2_UI)/(N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360);
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
                        const double _linear_movement_per_n_rounds,
                        const double _voltage_LNS, //[V]
                        const double _voltage_LPS, //[V]
                        const double _range,
                        const double _fullScalePot){
    
    DPRINT("Inizializzazione parametri");
    
    this.useUI=false;
    // max speed
    if(_maxSpeed_mm_s<=0){
        return -1;
    }
    maxSpeed_IU=speedfromMMsToIU(_maxSpeed_mm_s);
    
    // speed
    if(_speed_mm_s<=0 || _speed_mm_s>_maxSpeed_mm_s){
        return -2;
    }  
    speed_IU=speedfromMMsToIU(_speed_mm_s);
    
    // max acceleration
    if(_maxAcceleration_mm_s2<=0){
        return -4;
    }
    maxAcceleration_IU=accelerationfromMMs2ToIU(_maxAcceleration_mm_s2);
    
    // acceleration
    if(_acceleration_mm_s2<=0 || _acceleration_mm_s2>_maxAcceleration_mm_s2){
        return -5;
    }
    acceleration_IU=accelerationfromMMs2ToIU(_acceleration_mm_s2);
    
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
    
    // max high speed homing
    if(_maxHighSpeedHoming_mm_s<=0){
        return -8;
    }   
    maxHighSpeedHoming_IU=-speedfromMMsToIU(_maxHighSpeedHoming_mm_s);
    //maxHighSpeedHoming_mm_s = -_maxHighSpeedHoming_mm_s; // N.B. Dalla tastiera verra' inserito un numero positivo, 
                                               // che poi solo all'interno del drive ne verra' considerato 
                                               // solamente il segno opposto
    
    //  high speed homing
    if(_highSpeedHoming_mm_s<=0 || _highSpeedHoming_mm_s>_maxHighSpeedHoming_mm_s){
        return -9;
    }
    highSpeedHoming_IU=-speedfromMMsToIU(_highSpeedHoming_mm_s);
    
    
    // max low speed homing
    if(_maxLowSpeedHoming_mm_s<=0){
        return -10;
    }   
    maxLowSpeedHoming_IU = speedfromMMsToIU(_maxLowSpeedHoming_mm_s);
  
    
    // low speed homing
    if((_lowSpeedHoming_mm_s<=0) || (_lowSpeedHoming_mm_s>_maxLowSpeedHoming_mm_s)){
        return -11;
    }
    lowSpeedHoming_IU=speedfromMMsToIU(_lowSpeedHoming_mm_s);
    //lowSpeedHoming_mm_s=_lowSpeedHoming_mm_s;
    
    
    // ****************** max acceleration homing *********************
    if(_maxAccelerationHoming_mm_s2<=0){
        return -12;
    }
    maxAccelerationHoming_IU=accelerationfromMMs2ToIU(_maxAccelerationHoming_mm_s2);
    
    if(_accelerationHoming_mm_s2<=0 || _accelerationHoming_mm_s2>_maxAccelerationHoming_mm_s2){
        return -13;
    }
    accelerationHoming_IU = accelerationfromMMs2ToIU(_accelerationHoming_mm_s2); 
    
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
    DPRINT("VALORE INIZIALE n_encoder_lines: %f", n_encoder_lines);
    
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
    DPRINT("VALORE INIZIALE N_ROUNDS: %f", n_rounds);
    
    if(_linear_movement_per_n_rounds<=0){
        return -22;
    }
    linear_movement_per_n_rounds=_linear_movement_per_n_rounds;
    DPRINT("VALORE INIZIALE linear_movement_per_n_rounds: %f", linear_movement_per_n_rounds);
    
    axisID = _axisID;
    
    if(_voltage_LNS<0){
        return -23;
    }
    voltage_LNS = _voltage_LNS; 
        
    if(_voltage_LPS<0){
        return -24;
    }
    voltage_LPS = _voltage_LPS;     
        
    if(_range<=0){
        return -25;
    }
    range=_range;  
    
    if(_fullScalePot<=0){
        return -26;
    }
    fullScalePot=_fullScalePot;
    constantPot=_fullScalePot/65535;
    
    axisRef = TS_LoadSetup(setupFilePath.c_str());
    if(axisRef < 0){
        DERR("LoadSetup failed \"%s\", %s",setupFilePath.c_str(),TS_GetLastErrorText());
        return -27;
    }
    
    /*	Setup the axis based on the setup data previously, for axisID*/
    if(!TS_SetupAxis(_axisID, axisRef)){
        DERR("failed to setup axis %d, %s",axisID,TS_GetLastErrorText());
        return -28;
    }
   
    if(!TS_SelectAxis(_axisID)){
        DERR("failed to select axis %d, %s",_axisID,TS_GetLastErrorText());
        return -29;
    }
    
    /*	Execute the initialization of the drive (ENDINIT) */
    if(!TS_DriveInitialisation()){
        DERR("failed Low driver initialisation");
        return -30;
    }
    
     // Settare il registro per la lettura dell'encoder
    if(!TS_Execute("SCR=0x4338")){
        //descrErr=descrErr+" "+TS_GetLastErrorText()+". ";
        DERR("Failed TS_Execute command");
        return -31;
    }
  
    if(!TS_SetEventOnMotionComplete(0,0)){ 
	return -32;
    }
    
    readyState = true;
    internalHomingStateDefault=0;
    internalHomingStateHoming2=0;
    
    cap_position=0;
    controlLNS=true;
    
    // Gestione eventuale sospensione homing
    minimumIntervalForHoming=2; // s
    
    // Inizializzazione ultima volta esecuzione funzione homing
//    lastTimeTakenForHoming.tv_sec=0;
//    lastTimeTakenForHoming.tv_usec=0;
    
    gettimeofday(&lastTimeTakenForHoming,NULL);
    
    return 0;
}

int TechnoSoftLowDriver::homing(int mode){
    // Attenzione: la variabile mode non viene utilizzata
    
    double time_interval;
    struct timeval currentTimeTakenForHoming;
    
    gettimeofday(&currentTimeTakenForHoming,NULL);
    time_interval = ((double)currentTimeTakenForHoming.tv_sec+(double)currentTimeTakenForHoming.tv_usec/1000000.0)-((double)lastTimeTakenForHoming.tv_sec+(double)lastTimeTakenForHoming.tv_usec/1000000.0);
    
    if(time_interval>minimumIntervalForHoming){
       internalHomingStateDefault=0;
       internalHomingStateHoming2=0;
       controlLNS=true;
       cap_position=0; 
    }
    
    if(controlLNS){
        std::string descStr="";
        uint16_t contentRegMER=0;
        if((getStatusOrErrorReg(5, contentRegMER, descStr))<0){
            lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
            lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
            internalHomingStateDefault = 0;
            internalHomingStateHoming2=0;
            controlLNS=true;
            cap_position=0;
            return -1;
        }
        if(contentRegMER & ((uint16_t)1<<7)){
            // Il LNS e' attivo. Non c'e' bisogno di effettuare la procedura di homing.
            // Reset encoder e counter
            lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
            lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
            internalHomingStateDefault = 0;
            internalHomingStateHoming2=0;
            controlLNS=true;
            cap_position=0; 
            
            if(selectAxis()<0){
                return -2; 
            }
            
            if(resetEncoder()<0){
                return -3;
                   
            }
            if(resetCounter()<0){
                return -4;
            }
            return 0;
        }  
        controlLNS=false;    
    }
    uint16_t contentRegSRL=0;
    bool motionCompleted=false;
    
    if(mode==0){

        int risp;
        int switchTransited=0;
        
        std::string cappos = "CAPPOS";
        //long cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
        std::string descStr;
        //descStr.assign("");
        int absoluteMotionCompleted = 0;
        
    
        switch (internalHomingStateDefault) {
            case 0:
//                lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
//                lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
                if(selectAxis()<0){
                    
                    internalHomingStateDefault = 0;
                    controlLNS=true;
                    cap_position=0;
                    risp = -5;
                    break;
                }
//                if(setEventOnMotionComplete()<0){ 
//                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
//                        if(stopMotion()<0){
//                            risp= -9;
//                            break;
//                        }
//                        risp =-10;
//                        break;
//                } 
//                usleep(100000);
                
                if(moveVelocityHoming()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true;
                    if(stopMotion()<0){
                        risp = -6;
                        break;
                    }
                    
                    risp = -7;
                    break;
                }
                
                if(setEventOnLimitSwitch()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true;
                    if(stopMotion()<0){
                        risp = -8;
                        break;
                    }
                    risp = -9;
                    break;
                }
                
                DPRINT("STATE 0: event on limit switch activated ");
                
                DPRINT(" STATE 0: move velocity activated ");
                
                internalHomingStateDefault = 1;
                risp = 1;
                //stateHoming0++;
                break; 
            case 1:
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true;
                    risp = -10;
                    break;
                }
                if(checkEvent(switchTransited)<0){
                    internalHomingStateDefault = 0;// deve essere riinizializzato per successivi nuovi tentativi di homing 
                    controlLNS=true; 
                    if(stopMotion()<0){
                        risp = -11;
                        break;
                    }   
                    risp = -12;
                    break;
                } 
                DPRINT(" STATE 1: possible limit switch transition just checked ");
                if(switchTransited){
//                    if(setEventOnMotionComplete()<0){ 
//                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
//                        if(stopMotion()<0){
//                            risp= -9;
//                            break;
//                        }
//                        risp =-10;
//                        break;
//                    }  
//                    if(setEventOnMotionComplete()<0){ 
//                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
//                        if(stopMotion()<0){
//                            risp= -9;
//                            break;
//                        }
//                        risp =-10;
//                        break;
//                    } 
//                    if(getLVariable(cappos, cap_position)<0){ 
//    //                if(stopMotion()<0){
//    //                    return -13;
//    //                }
//                        internalHomingStateDefault=0;
//                        risp = -14;
//                        break;
//                    }
                    //eventOnMotionCompleteSet = true;
                    internalHomingStateDefault=2;
                    DPRINT(" STATE 1: Negative limit switch transited. Event on motion completed set ");
//                    double capturePositionHoming;
//                    capturePositionHoming = (linear_movement_per_n_rounds*cap_position)/(steps_per_rounds*n_rounds*const_mult_technsoft);
                    //DPRINT("************** switch transitato. cap_position in mm: %f**************",capturePositionHoming);
                    //cap_position=0; //  ********** lasciamolo cosi************
                    //usleep(5000);
                }
                // **************DA IMPLEMENTARE:*****************
                // RESET ON Limit Switch Transition Event
                risp = 1;
                //stateHoming1++;
                break; 
            case 2:

                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true; 
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -11;
                        break;
                    }
                    risp = -13;
                    break;
                }
                
//                if(getLVariable(cappos, cap_position)<0){ 
//    //                if(stopMotion()<0){
//    //                    return -13;
//    //                }
//                        internalHomingStateDefault=0;
//                        risp = -14;
//                        break;
//                }
                
//                if(checkEvent(motionCompleted)<0){
//                    internalHomingStateDefault = 0;
//                    if(stopMotion()<0){
//                        risp = -12;
//                        break;
//                    }
//                    risp= -13;
//                    break;
//                }
                if((getStatusOrErrorReg(3, contentRegSRL, descStr))<0){
                    controlLNS=true; 
                    internalHomingStateDefault = 0;
//                    descStr=descStr+"Unknown status. ";
//                    ERR("Reading state error: %s",descStr.c_str());
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -14;
                        break;
                    }
                    risp= -15;
                    break;
                }
                if((contentRegSRL & ((uint16_t)1<<10))){
                    motionCompleted=true;
                }
                //contentRegSRL=0;
                DPRINT("************** STATE 2: possible event on motion completed checked **************");
                if(motionCompleted){
                    DPRINT("************** STATE 2: Motion completed after transition **************"); 
                    internalHomingStateDefault = 3;
//                    double positionHoming;
//                    getEncoder(&positionHoming);
                    //DPRINT("************** STATE 2 position in mm after motion is completed: %f **************",positionHoming);
                    //sleep(60);
                }
                if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                    risp = -16;
                    break;
                }
                risp= 1;
                //stateHoming2++;
                break;
            case 3:
                //usleep(10000000); // Garantisce che il motore sia veramente fermo
                
                // The motor is not in motion
                //DPRINT("************** STATE 3: read the captured position on limit switch transition**************");
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true; 
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -17;
                        break;
                    }
                    risp = -18;
                    break;
                }
                
                if(getLVariable(cappos, cap_position)<0){ 
    //                if(stopMotion()<0){
    //                    return -13;
    //                }
                    internalHomingStateDefault=0;
                    controlLNS=true; 
                    risp = -18;
                    break;
                }
                

//                while (1){
//                double positionState3;
//                getEncoder(&positionState3);
                //DPRINT("************** STATE 3 position in mm prima del recupero: %f **************",positionState3);
                //usleep(1000000);
//                }
                
//                if(getLVariable(cappos, cap_position)<0){ 
//    //                if(stopMotion()<0){
//    //                    return -13;
//    //                }
//                    internalHomingStateDefault=0;
//                    risp = -14;
//                    break;
//                }
                //DPRINT("************** STATE 3: the captured position on limit switch transition is %ld [drive internal position units]**************",cap_position);
//                if(setEventOnLimitSwitch()<0){
//                    internalHomingStateDefault = 0;
//                    if(stopMotion()<0){
//                        risp = -4;
//                        break;
//                    }
//                    risp = -5;
//                    break;
//                }
//                usleep(5000);
//                if(setEventOnMotionComplete()<0){ 
//                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
//                        if(stopMotion()<0){
//                            risp= -9;
//                            break;
//                        }
//                        risp =-10;
//                        break;
//                    }  
//                usleep(10000);
            /*	Command an absolute positioning on the captured position */
                
                
                if(moveAbsoluteStepsHoming(cap_position)<0){  
                    internalHomingStateDefault=0;
                    controlLNS=true; 
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -19;
                        break;
                    }
                    risp = -20;
                    break;
                }
                
                if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                    risp = -21;
                    break;
                }
                
                //sleep(180);//**********************************************DA TOGLIERE*****************
                DPRINT("************** STATE 3: command of absolute positioning on the captured position sended **************");
                internalHomingStateDefault = 4;
                risp= 1;
                //stateHoming3++;
                //sleep(20);
                break;
            case 4:
//                DPRINT("************** STATE 4: cap_position in mm: %f**************",cap_position);
//                sleep(30);
                DPRINT("************** STATE 4: wait for positioning to end **************");
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true; 
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -22;
                        break;
                    }
                    risp = -23;
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
//                  
                
                if((getStatusOrErrorReg(3, contentRegSRL, descStr))<0){
//                    descStr=descStr+"Unknown status. ";
//                    ERR("Reading state error: %s",descStr.c_str());
                    controlLNS=true; 
                    internalHomingStateDefault = 0;
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -25;
                        break;
                    }
                    risp= -24;
                    break;
                }
                if((contentRegSRL & ((uint16_t)1<<10))){
                    absoluteMotionCompleted=true;
                }
                
//                if(checkEvent(absoluteMotionCompleted)<0){
//                    internalHomingStateDefault = 0;
//                    if(stopMotion()<0){
//                    //eventOnMotionCompleteSet = false;
//                        risp= -17;
//                        break;
//                    }
//                //eventOnMotionCompleteSet = false;
//                    risp= -18;
//                    break;
//                }
  
                if(absoluteMotionCompleted){
                    internalHomingStateDefault = 5;
                    
//                    DPRINT("************** STATE 4: motor positioned to end **************");
//                    double positionHoming;
//                    getEncoder(&positionHoming);
//                    DPRINT("************** STATE 4 current position in mm: %f **************",positionHoming);
                }
            // **************DA IMPLEMENTARE:*****************
            // RESET ON Event On Motion Complete
                if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                    risp = -25;
                    break;
                }
                
                risp= 1;
                //stateHoming4++;
                
                break;
            case 5:
                // The motor is positioned to end
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true; 
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -26;
                        break;
                    }
                    risp = -27;
                    break;
                }
                
                cap_position=0;
                usleep(1000000);
                
                if(resetEncoder()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true; 
                    //if(stopMotion()<0){
                    //return -23;
                    //}
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -28;
                        break;
                    }
                    risp= -29;
                    break;
                }
                if(resetCounter()<0){
                    internalHomingStateDefault = 0;
                    controlLNS=true; 
                    //if(stopMotion()<0){
                    //return -25;
                    //}
                    if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -30;
                        break;
                    }
                    risp= -31;
                    break;
                }
                if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                        risp = -32;
                        break;
                }
                DPRINT("************** STATE 5: encoder e counter e counter are reset **************");
                internalHomingStateDefault = 0;
                risp= 0;
                controlLNS=true;
                break;
            default:
                internalHomingStateDefault = 0;
                controlLNS=true;
                if(!TS_SetEventOnLimitSwitch(LSW_NEGATIVE, TRANSITION_HIGH_TO_LOW, FALSE, FALSE)){
                    risp = -33;
                    break;
                }
                risp= -34;
                break; 
        } 
        lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
        lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
        return risp;
    }
    else if(mode==1){
        int risp;
        uint16_t contentReg;
        std::string descStr = "";
        short homingDone = 0;
        double time_interval;
        struct timeval currentTimeTakenForHoming;

        switch (internalHomingStateHoming2) {
            case 0:       
                DPRINT("************** Homing procedure Homing2. STATE 0. **************");
                //gettimeofday(&lastTimeTakenForHoming,NULL);
                
                if(moveVelocityHoming()<0){
                    internalHomingStateHoming2=0;
                    controlLNS=true;
                    risp= -35;
                    break;
                }
                internalHomingStateHoming2=1;
                risp= 1;
                break;
            case 1:
                DPRINT("************** Homing procedure Homing2. STATE 1. **************");
                
//                gettimeofday(&currentTimeTakenForHoming,NULL);
//                time_interval = ((double)currentTimeTakenForHoming.tv_sec+(double)currentTimeTakenForHoming.tv_usec/1000000.0)-((double)lastTimeTakenForHoming.tv_sec+(double)lastTimeTakenForHoming.tv_usec/1000000.0);
//                // Aggiornamento lastTimeTakenForHoming
//                lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
//                lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
                
//                if(time_interval>minimumIntervalForHoming){ // In questo caso la procedura di homing sta riprendendo dopo una sospensione.
//                                                            // Potrebbe essere causata ad esempio da un comando di stop.
//                    
//                    // Ma come mi richiami per la prima volta in questo stato, dopo tanto tempo, anziche' nello stato iniziale?
//                    // Le operazioni di questo stato non dovranno essere eseguite! 
//                    // Perche' lo stato fisico delle cose potrebbe essere cambiato.
//                    // Dobbiamo quindi far ripartire la procedura di homing 
//                    // come se fosse la prima volta che la procedura venisse chiamata,
//                    // in modo del tutto trasparente alla control unit
//                    
//                    internalHomingStateHoming2 = 0;
//                    controlLNS = true;
//                    //perche' deve avvenire in modo del tutto trasparente alla control unit:
//                    risp = 1;
//                    break;
//                }
//                DPRINT("time interval: %f",time_interval);

                if((getStatusOrErrorReg(5, contentReg, descStr))<0){
                    //ERR("Reading state error: %s",descStr.c_str());
                    internalHomingStateHoming2=0;
                    controlLNS=true;
//                    if(stopMotion()<0){
//                        risp= -36;
//                        break;
//                    }
                    risp= -37;
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
                DPRINT("************** STATE 2: wait for stopping motor after switch transition**************");
                if(selectAxis()<0){
                    internalHomingStateHoming2= 0;
                    controlLNS=true; 
                    risp = -38;
                    break;
                }
                if((getStatusOrErrorReg(3, contentRegSRL, descStr))<0){
//                    descStr=descStr+"Unknown status. ";
//                    ERR("Reading state error: %s",descStr.c_str());
                    controlLNS=true; 
                    internalHomingStateHoming2 = 0;
                    risp= -39;
                    break;
                }
                
                if((contentRegSRL & ((uint16_t)1<<10))){
                    motionCompleted=true;
                }
  
                if(motionCompleted){
                    internalHomingStateHoming2 = 3;
                    
//                    DPRINT("************** STATE 4: motor positioned to end **************");
//                    double positionHoming;
//                    getEncoder(&positionHoming);
//                    DPRINT("************** STATE 4 current position in mm: %f **************",positionHoming);
                }
                risp= 1;
                break;
            case 3:
                DPRINT("************** Homing procedure Homing2. STATE 2. **************");
                DPRINT("************** Reset encoder e counter **************");
                
//                gettimeofday(&currentTimeTakenForHoming,NULL);
//                time_interval = ((double)currentTimeTakenForHoming.tv_sec+(double)currentTimeTakenForHoming.tv_usec/1000000.0)-((double)lastTimeTakenForHoming.tv_sec+(double)lastTimeTakenForHoming.tv_usec/1000000.0);
//                // Aggiornamento lastTimeTakenForHoming
//                lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
//                lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;

                // Ma come mi richiami per la prima volta in questo stato, dopo tanto tempo, anziche' nello stato iniziale?
                // Le operazioni di questo stato non dovranno essere eseguite! 
                // Perche' lo stato fisico delle cose potrebbe essere cambiato.
                // Dobbiamo quindi far ripartire la procedura di homing 
                // come se fosse la prima volta che la procedura venisse chiamata,
                // in modo del tutto trasparente alla control unit
//                if(time_interval>minimumIntervalForHoming){ // In questo caso la procedura di homing sta riprendendo dopo una sospensione.
//                                                            // Potrebbe essere causata ad esempio da un comando di stop.
//                    
//                    // Ma come mi richiami per la prima volta in questo stato, dopo tanto tempo, anziche' nello stato iniziale?
//                    // Le operazioni di questo stato non dovranno essere eseguite! 
//                    // Perche' lo stato fisico delle cose potrebbe essere cambiato.
//                    // Dobbiamo quindi far ripartire la procedura di homing 
//                    // come se fosse la prima volta che la procedura venisse chiamata,
//                    // in modo del tutto trasparente alla control unit
//                    
//                    internalHomingStateHoming2 = 0;
//                    controlLNS = true;
//                    // in modo trasparente alla control unit:
//                    risp = 1;
//                    // Aggiornamento lastTimeTakenForHoming
////                    lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
////                    lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
//                    break;
//                }
                DPRINT("time interval: %f",time_interval);
                usleep(1000000);
                
                if(resetEncoder()<0){
                    internalHomingStateHoming2=0;
                    controlLNS=true;
//                    if(stopMotion()<0){
//                        risp= -38;
//                        break;
//                    }
                    risp= -40;
                    break;
                }
                if(resetCounter()<0){
                    internalHomingStateHoming2=0;
                    controlLNS=true;
//                    if(stopMotion()<0){
//                        risp= -40;
//                        break;
//                    }
                    risp= -41;
                    break;
                }
//                lastTimeTakenForHoming.tv_sec=0; // Solo per lasciare pulita la memoria per molto tempo...
//                lastTimeTakenForHoming.tv_usec=0;     
                internalHomingStateHoming2=0;
                controlLNS=true;
                risp=0;
                break;
            default:
//                lastTimeTakenForHoming.tv_sec=0;
//                lastTimeTakenForHoming.tv_usec=0;
                internalHomingStateHoming2 = 0;
                controlLNS=true;
                risp=-42;
                break;
        }
        lastTimeTakenForHoming.tv_sec = currentTimeTakenForHoming.tv_sec;
        lastTimeTakenForHoming.tv_usec= currentTimeTakenForHoming.tv_usec;
        return risp;
    }
    else{
        internalHomingStateHoming2=0;               
        controlLNS=true;
        lastTimeTakenForHoming.tv_sec=currentTimeTakenForHoming.tv_sec;
        lastTimeTakenForHoming.tv_usec=currentTimeTakenForHoming.tv_usec;
        return -100;
    }
}

int TechnoSoftLowDriver::moveRelativeSteps(const long& deltaPosition){ // Inteso come comando
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -2;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
//        lastTimeTakenForHoming.tv_sec=0;
//        lastTimeTakenForHoming.tv_usec=0;
    }
    
    DPRINT("Velocita' movimentazione relativa: %f",speed_IU);
    DPRINT("Accelerazione movimentazione relativa: %f",acceleration_IU);
    
    if(!TS_MoveRelative(deltaPosition, speed_IU, acceleration_IU, isAdditive, movement, referenceBase)){
        DERR("error relative moving");
        return -3;
    }
    return 0;
}

double TechnoSoftLowDriver::getdeltaMicroSteps(const double& deltaMillimeters){
    
    return round((steps_per_rounds*n_rounds*const_mult_technsoft*deltaMillimeters)/linear_movement_per_n_rounds);
}

int TechnoSoftLowDriver::setSpeed(const double& _speed_mm_s){ // _speed_mm_s [mm/s] 
    
    if(_speed_mm_s<0){
        DERR("Speed = %f <0 mm/s",_speed_mm_s);
        return -1;
    }
    
    double _speed_UI = speedfromMMsToIU(_speed_mm_s);
    
    if(_speed_UI>maxSpeed_IU){
        DERR("Speed = %f IU > maxSpeed",_speed_UI);
        return -2;
    }
    
    speed_IU = _speed_UI;
    
    DPRINT("valore della speed settata: %f", _speed_UI);
    return 0;
}

int TechnoSoftLowDriver::getSpeed(double& _speed_mms){
    
    // Conversione speed_IU in mm/s
    _speed_mms=speedfromIUTOMMs(speed_IU);
    return 0;
}

int TechnoSoftLowDriver::setMeasureUnit(const bool& inSteps)
{
 this->useUI=inSteps;
}

int TechnoSoftLowDriver::setMaxSpeed(const double& _maxspeed_mm_s){

    //DPRINT("setMaxSpeed: _maxspeed_mm_s=%f,speed_mm_s=%f",_maxspeed_mm_s,speed_mm_s);
    if(_maxspeed_mm_s<=0){
        DERR("max speed= %f <=0 mm/s",_maxspeed_mm_s);
        return -1;
    }
    double _maxSpeed_IU = speedfromMMsToIU(_maxspeed_mm_s);
    
    if(_maxSpeed_IU<speed_IU){
        DERR("Max speed = %f IU < current speed",_maxSpeed_IU);
        return -2;
    }
    
    maxSpeed_IU = _maxSpeed_IU;
    return 0;
}

int TechnoSoftLowDriver::getMaxSpeed(double& _maxspeed_mms){
    
    // Conversione speed_IU in mm/s
    _maxspeed_mms=speedfromIUTOMMs(maxSpeed_IU);
    return 0;    
}

int TechnoSoftLowDriver::setAcceleration(const double& _acceleration_mm_s2){
    
    if(_acceleration_mm_s2<=0){
        DERR("(_acceleration_mm_s2 = %f <=0 mm/s^2",_acceleration_mm_s2);
        return -1;
    }
            
    double _acceleration_UI = accelerationfromMMs2ToIU(_acceleration_mm_s2);
    
    if(_acceleration_UI>maxAcceleration_IU){
        DERR("Acceleration = %f IU > maxAcceleration",_acceleration_UI);
        return -2;
    }
    
    acceleration_IU = _acceleration_UI;
    return 0;
}

int TechnoSoftLowDriver::getAcceleration(double& _acceleration_mms){
    
    _acceleration_mms=accelerationfromIUToMMs2(acceleration_IU);
    return 0;
}

int TechnoSoftLowDriver::setMaxAcceleration(const double& _maxacceleration_mm_s2){

    if(_maxacceleration_mm_s2<=0){
        DERR("_acceleration_mm_s2 = %f <=0 mm/s^2",_maxacceleration_mm_s2);
        return -1;
    }
    double _maxAcceleration_UI = accelerationfromMMs2ToIU(_maxacceleration_mm_s2);
    
    if(_maxAcceleration_UI<maxAcceleration_IU){
        DERR("_maxAcceleration_UI<maxAcceleration_IU");
        return -2;
    }
    maxAcceleration_IU= _maxAcceleration_UI;
    return 0;
}

int TechnoSoftLowDriver::getMaxAcceleration(double& _maxacceleration_mms){
    
    _maxacceleration_mms=accelerationfromIUToMMs2(maxAcceleration_IU);
    return 0;
}

int TechnoSoftLowDriver::setAdditive(const BOOL& _isAdditive){
    //DPRINT("Chiamata setAdditive");
    if((_isAdditive!=TRUE) && (_isAdditive!=FALSE)){
        DERR("setAdditive= %d",_isAdditive);
        return -1;
    }
    isAdditive = _isAdditive;
    return 0;
}

int TechnoSoftLowDriver::getAdditive(BOOL& _isAdditive){
    //DPRINT("Chiamata setAdditive");
    
    _isAdditive=isAdditive;
    return 0;
}

int TechnoSoftLowDriver::setMovement(const short& _movement){
    //DPRINT("Chiamata setMovement");
    if((_movement!=UPDATE_NONE) && (_movement!=UPDATE_IMMEDIATE) && (_movement!=UPDATE_ON_EVENT)){
        DERR("setMovement = %d",_movement);
        return -1;
    }
    movement = _movement;
    return 0;
}

int TechnoSoftLowDriver::getMovement(short& _movement){
    //DPRINT("Chiamata setAdditive");
    _movement=movement;
    return 0;
}

int TechnoSoftLowDriver::setReferenceBase(const short& _referenceBase){
    //DPRINT("Chiamata setReferenceBase");
    if((_referenceBase!=FROM_MEASURE) && (_referenceBase!=FROM_REFERENCE)){
        DERR("_referenceBase = %d",_referenceBase);
        return -1;
    }
    referenceBase=_referenceBase;
    return 0;
}

int TechnoSoftLowDriver::getReferenceBase(short& _referenceBase){
    //DPRINT("Chiamata setReferenceBase");

    _referenceBase=referenceBase;
    return 0;
}

// Set homing parameters
int TechnoSoftLowDriver::sethighSpeedHoming(const double& _highSpeedHoming_mm_s){
    
    if(_highSpeedHoming_mm_s<=0){
        return -1;
    }
    
    double _highSpeedHoming_IU = speedfromMMsToIU(_highSpeedHoming_mm_s);
    
    if(_highSpeedHoming_IU>(-maxHighSpeedHoming_IU)){
        return -2;
    }

    highSpeedHoming_IU = -_highSpeedHoming_IU;
    return 0;
}

int TechnoSoftLowDriver::getHighSpeedHoming(double& _highSpeedHoming_mm_s){
    
    // Conversione speed_IU in mm/s
    _highSpeedHoming_mm_s=std::abs(speedfromIUTOMMs(highSpeedHoming_IU));
    return 0;
}

int TechnoSoftLowDriver::setMaxhighSpeedHoming(const double& _maxhighSpeedHoming_mm_s){

    if(_maxhighSpeedHoming_mm_s<=0){
        return -1;
    }
    
    double _maxHighSpeedHoming_IU = speedfromMMsToIU(_maxhighSpeedHoming_mm_s);
    
    if(_maxHighSpeedHoming_IU<-highSpeedHoming_IU || _maxHighSpeedHoming_IU<maxLowSpeedHoming_IU){
        return -2;
    }

    maxHighSpeedHoming_IU = _maxHighSpeedHoming_IU;
    return 0;
}

int TechnoSoftLowDriver::getMaxhighSpeedHoming(double& _maxhighSpeedHoming_mm_s){
    
    // Conversione speed_IU in mm/s
    _maxhighSpeedHoming_mm_s=std::abs(speedfromIUTOMMs(maxHighSpeedHoming_IU));
    return 0;
}

int TechnoSoftLowDriver::setlowSpeedHoming(const double& _lowSpeedHoming_mm_s){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    //DPRINT("Chiamata setlowSpeedHoming");
    
    if(_lowSpeedHoming_mm_s<=0){
        return -1;
    }
    double _lowSpeedHoming_IU = speedfromMMsToIU(_lowSpeedHoming_mm_s);
    
    if(_lowSpeedHoming_IU>maxLowSpeedHoming_IU){
        return -2;
    }
//    if(_lowSpeedHoming_mm_s<=0 || _lowSpeedHoming_mm_s>maxLowSpeedHoming_mm_s){
//        return -1;
//    }
    lowSpeedHoming_IU = _lowSpeedHoming_IU;
    return 0;
}

int TechnoSoftLowDriver::getlowSpeedHoming(double& _lowSpeedHoming_mm_s){
    
    // Conversione speed_IU in mm/s
    _lowSpeedHoming_mm_s=speedfromIUTOMMs(lowSpeedHoming_IU);
    return 0;
}

int TechnoSoftLowDriver::setMaxlowSpeedHoming(const double& _maxlowSpeedHoming_mm_s){

    if(_maxlowSpeedHoming_mm_s<=0){
        return -1;
    }
    double _maxLowSpeedHoming_IU = speedfromMMsToIU(_maxlowSpeedHoming_mm_s);
    
    if(_maxLowSpeedHoming_IU<lowSpeedHoming_IU || _maxLowSpeedHoming_IU>maxHighSpeedHoming_IU){
        return-2;
    }

    maxLowSpeedHoming_IU = _maxLowSpeedHoming_IU;
    return 0;
}

int TechnoSoftLowDriver::getMaxlowSpeedHoming(double& _maxlowSpeedHoming_mm_s){
    
    // Conversione speed_IU in mm/s
    _maxlowSpeedHoming_mm_s=speedfromIUTOMMs(maxLowSpeedHoming_IU);
    return 0;
}

int TechnoSoftLowDriver::setaccelerationHoming(const double&  _accelerationHoming_mm_s2){

    if(_accelerationHoming_mm_s2<=0){
        return -1;
    }
    double _accelerationHoming_IU =  accelerationfromMMs2ToIU(_accelerationHoming_mm_s2);  
        
    if(_accelerationHoming_IU>maxAccelerationHoming_IU){
        return -2;
    }
    accelerationHoming_IU = _accelerationHoming_IU;
    return 0;
}

int TechnoSoftLowDriver::getaccelerationHoming(double&  _accelerationHoming_mm_s2){
    
    _accelerationHoming_mm_s2=accelerationfromIUToMMs2(accelerationHoming_IU);
    return 0;
}

int TechnoSoftLowDriver::setMaxAccelerationHoming(const double&  _maxAccelerationHoming_mm_s2){
    //printf("acceleration = %f, max acceleration = %f", _acceleration,maxAcceleration);
    
    if(_maxAccelerationHoming_mm_s2<=0){
        return -1;
    }
    
    double _maxAccelerationHoming_IU = accelerationfromMMs2ToIU(_maxAccelerationHoming_mm_s2);
    
    if(_maxAccelerationHoming_IU <accelerationHoming_IU){
        return -2;
    }
    maxAccelerationHoming_IU = _maxAccelerationHoming_IU;
    return 0;
}

int TechnoSoftLowDriver::getMaxAccelerationHoming(double&  _maxAccelerationHoming_mm_s2){
    //printf("acceleration = %f, max acceleration = %f", _acceleration,maxAcceleration);
    
    _maxAccelerationHoming_mm_s2=accelerationfromIUToMMs2(maxAccelerationHoming_IU);
    return 0;
}

// Set encoder lines
int TechnoSoftLowDriver::setEncoderLines(const double& _encoderLines){
    //DPRINT("Chiamata setEncoderLines");
    if(_encoderLines<=0){
        return -1;
    }
    n_encoder_lines=_encoderLines;
    return 0;
}

int TechnoSoftLowDriver::getEncoderLines(double& _encoderLines){
    //DPRINT("Chiamata setEncoderLines");
    _encoderLines=n_encoder_lines;
    return 0;
}

int TechnoSoftLowDriver::setConst_mult_technsoft(const double& _const_mult_technsoft){
    //DPRINT("Chiamata setConst_mult_technsoft");
    if(_const_mult_technsoft<=0){
        return -1;
    }
    const_mult_technsoft=_const_mult_technsoft;
    return 0;
}

int TechnoSoftLowDriver::getConst_mult_technsoft(double& _const_mult_technsoft){
    //DPRINT("Chiamata setConst_mult_technsoft");
    
    _const_mult_technsoft=const_mult_technsoft;
    return 0;
}

int TechnoSoftLowDriver::setSteps_per_rounds(const double& _steps_per_rounds){
    //DPRINT("Chiamata setSteps_per_rounds");
    if(_steps_per_rounds<=0){
        return -1;
    }
    steps_per_rounds=_steps_per_rounds;
    return 0;
}

int TechnoSoftLowDriver::getSteps_per_rounds(double& _steps_per_rounds){
    //DPRINT("Chiamata setSteps_per_rounds");
    _steps_per_rounds=steps_per_rounds;
    return 0;
}

int TechnoSoftLowDriver::setN_rounds(const double& _n_rounds){
    //DPRINT("Chiamata setN_rounds");
    if(_n_rounds<=0){
        return -1;
    }
    n_rounds=_n_rounds;
    return 0;
}

int TechnoSoftLowDriver::getN_rounds(double& _n_rounds){
    //DPRINT("Chiamata setN_rounds");
    
    _n_rounds=n_rounds;
    return 0;
}

int TechnoSoftLowDriver::setLinear_movement_per_n_rounds(const double& _linear_movement_per_n_rounds){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    if(_linear_movement_per_n_rounds<=0){
        return -1;
    }
    linear_movement_per_n_rounds=_linear_movement_per_n_rounds;
    return 0;
}

int TechnoSoftLowDriver::getLinear_movement_per_n_rounds(double& _linear_movement_per_n_rounds){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
//    if(_linear_movement_per_n_rounds<=0){
//        return -1;
//    }
    _linear_movement_per_n_rounds=linear_movement_per_n_rounds;
    return 0;
}

int TechnoSoftLowDriver::setvoltage_LNS(const double& _voltage_LNS){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    if(_voltage_LNS<0){
        return -1;
    }
    voltage_LNS=_voltage_LNS;
    return 0;
}

int TechnoSoftLowDriver::getvoltage_LNS(double& _voltage_LNS){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    _voltage_LNS=voltage_LNS;
    return 0;
}

int TechnoSoftLowDriver::setvoltage_LPS(const double& _voltage_LPS){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    if(_voltage_LPS<0){
        return -1;
    }
    voltage_LPS=_voltage_LPS;
    return 0;
}

int TechnoSoftLowDriver::getvoltage_LPS(double& _voltage_LPS){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    
    _voltage_LPS=voltage_LPS;
    return 0;
}

int TechnoSoftLowDriver::setRange(const double& _range){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    if(_range<0){
        return -1;
    }
    range=_range;
    return 0;
}

int TechnoSoftLowDriver::getRange(double& _range){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    _range=range;
    return 0;
}

int TechnoSoftLowDriver::setFullscalePot(const double& _fullScale){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    if(_fullScale<0){
        return -1;
    }
    fullScalePot=_fullScale;
    constantPot=_fullScale/65535;
    return 0;
}

int TechnoSoftLowDriver::getFullscalePot(double& _fullScale){
    //DPRINT("Chiamata setLinear_movement_per_n_rounds");
    
//    fullScalePot=_fullScale;
    _fullScale=fullScalePot;
    return 0;
}


int TechnoSoftLowDriver::moveAbsoluteSteps(const long& absPosition){ // Inteso come comando
    

    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -2;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
    
    if(!TS_MoveAbsolute(absPosition, speed_IU, acceleration_IU, movement, referenceBase)){
        DERR("error absolute step moving");
        return -3;
    }
    return 0;
}

int TechnoSoftLowDriver::moveVelocityHoming(){
    
    if(!TS_MoveVelocity(highSpeedHoming_IU, accelerationHoming_IU, movementHoming, referenceBaseHoming)){
        DERR("(homing) Error moving velocity ");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::moveAbsoluteStepsHoming(const long& absPosition) const{
    
    if(!TS_MoveAbsolute(absPosition, lowSpeedHoming_IU, accelerationHoming_IU, movementHoming, referenceBaseHoming)){
        DERR("(homing) Error absolute steps moving");
        return -2;
    }
    return 0;
}

int TechnoSoftLowDriver::stopMotion(){
    

    if(!TS_Stop()){
        return -1;
    }
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
//        lastTimeTakenForHoming.tv_sec=0;
//        lastTimeTakenForHoming.tv_usec=0;
    }
    DPRINT("Motor with axis = %d is stopped, %s",axisID, TS_GetLastErrorText());
    return 0;
}

int TechnoSoftLowDriver::providePower(){ //******** Inteso come comando *********
    DPRINT("provide power to axis:%d",axisID);
//      if(!TS_SelectAxis(axisID)){
//        ERR("ALEDEBUG Error selecting axis");
//        return -1;
//    }
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -1;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
//        lastTimeTakenForHoming.tv_sec=0;
//        lastTimeTakenForHoming.tv_usec=0;
    }
    
    if(!TS_Power(POWER_ON)){
        //ERR("ALEDEBUG Error selecting axis");
        return -2;
    }
				
    /*	Wait for power stage to be enabled */
    WORD sAxiOn_flag = 0;
    int maxCount=20;
    int count=0;
    while(sAxiOn_flag == 0 && count<maxCount){
        //DPRINT("CHECKING POWER ON COMPLETED");
        /* Check the status of the power stage */
        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
	    
            //ERR("ALEDEBUG Error TS_ReadStatus");
            return -3;
        }
        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
        count++;
        usleep(5000);
    }
    //DPRINT("ALEDEBUG correctly powered on");
    //poweron=true;
    
    return 0;
}

int TechnoSoftLowDriver::stopPower(){ //******** Inteso come comando *********
    //DPRINT("stopping power to axis:%d",axisID);
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -1;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
//        lastTimeTakenForHoming.tv_sec=0;
//        lastTimeTakenForHoming.tv_usec=0;
    }
    
    if(!TS_Power(POWER_OFF)){
        return -2;
    }
    DPRINT("Motor with axis id = %d is power off",axisID);
    
    return 0;
}

int TechnoSoftLowDriver::deinit(){ // Identical to TechnoSoftLowDriver::stopPower()
    
    DPRINT("TechnoSoftLowDriver %d object is deallocated",axisID);
    return 0;
}

int TechnoSoftLowDriver::getCounter(double* deltaPosition_mm){
    
    DPRINT("Reading COUNTER position");
 
    long tposition;

    if(!TS_GetLongVariable("TPOS", tposition)){
        return -1;
    }
    *deltaPosition_mm = (tposition*linear_movement_per_n_rounds)/(steps_per_rounds*const_mult_technsoft*n_rounds);
    return 0;
}

int TechnoSoftLowDriver::getPotentiometer(double* deltaPosition_mm){
    
    DPRINT("Reading potentiometer");

    short valueAd2;

    if(TS_GetIntVariable("CSPD",valueAd2)<0){
        return -1;
    }
    
    double voltage=(valueAd2*constantPot)-10; //[V]
    * deltaPosition_mm=(((voltage-voltage_LNS)/(voltage_LPS-voltage_LNS))*range); 
    
    //*deltaPosition_mm = (tposition*linear_movement_per_n_rounds)/(steps_per_rounds*const_mult_technsoft*n_rounds);
    return 0;
} 

int TechnoSoftLowDriver::getEncoder(double* deltaPosition_mm){
    
    DPRINT("Reading ENCODER position");
    
    long aposition;
    if(!TS_GetLongVariable("APOS", aposition)){
        return -1;
    }
    //DPRINT("Final APOS for homing %ld",aposition);
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

int TechnoSoftLowDriver::getStatusOrErrorReg(const short& regIndex, WORD& contentRegister, std::string& descrErr){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    if(!TS_ReadStatus(regIndex,contentRegister)){

        //DERR("Error at the register reading: %s",TS_GetLastErrorText());
        //descrErr.assign(TS_GetLastErrorText());
        descrErr=descrErr+" Error reading status: "+TS_GetLastErrorText();
        return -1;
    }
    return 0;
}

int TechnoSoftLowDriver::getEmergency(BYTE nIO, BYTE& inValue, std::string& descrErr){
    
    if(!TS_GetInput(nIO, inValue)){
        descrErr=descrErr+" Error reading status: "+TS_GetLastErrorText();
        return -1;   
    }
    return 0;
}

int TechnoSoftLowDriver::resetFault(){ // Considerato come COMANDO
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -1;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
//        lastTimeTakenForHoming.tv_sec=0;
//        lastTimeTakenForHoming.tv_usec=0;
    }
    
    if(!TS_ResetFault()){
         return -2;
         // Note: the drive-motor will return to FAULT status (SRH.15=1) if there are
         // errors when the function is executed)
    }
 
    return 0;
}

int TechnoSoftLowDriver::hardreset(bool mode){
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -1;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
//        lastTimeTakenForHoming.tv_sec=0;
//        lastTimeTakenForHoming.tv_usec=0;
    }
    
    if(mode){
        if(!TS_Save()){
            return -2;
        }
    }
    
    if(!TS_Reset()){
        return -3;
    }
    
    /*	Execute the initialization of the drive (ENDINIT) */
    if(!TS_DriveInitialisation()){
        DERR("failed Low driver initialisation");
        return -4;
    }
    
     // Settare il registro per la lettura dell'encoder
    if(!TS_Execute("SCR=0x4338")){
        //descrErr=descrErr+" "+TS_GetLastErrorText()+". ";
        DERR("Failed TS_Execute command");
        return -5;
    }
    
    return 0;
}

int TechnoSoftLowDriver::selectAxis(){
     
    if(!TS_SelectAxis(axisID)){
        DERR("failed to select axis %d %s",axisID, TS_GetLastErrorText());
        return -1;
    }
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

/*****************************************************************/
/*****************************************************************/
 void SerialCommChannelTechnosoft::PrintChannel()
{
  DPRINT("%d %s %d %d\n",this->fd, this->pszDevName.c_str(),this->btType,this->baudrate);
}
/*****************************************************************/
 int  SerialCommChannelTechnosoft::getFD() {return this->fd;}
