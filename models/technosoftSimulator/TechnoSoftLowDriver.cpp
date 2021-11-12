#include "TechnoSoftLowDriver.h"
#include <common/debug/core/debug.h>
#include <iostream>
#include <sys/time.h>

using namespace common::actuators::models::simul;

SerialCommChannelTechnosoft::SerialCommChannelTechnosoft(int hostID, const std::string& pszDevName,BYTE btType,DWORD baudrate){
    init(hostID, pszDevName,btType,baudrate); // La funzione init non ritorna mai un numero negativo per quello che fa, quindi
                                       // e' inutile mettere il controllo. E poi se ritornasse un numero negativo bisognerebbe lanciare
                                      // una eccezione dato che si tratta di un metodo costruttore
}

std::string  SerialCommChannelTechnosoft::getDevName(){return this->pszDevName;}
int SerialCommChannelTechnosoft::getbtType(){return this->btType;}
int SerialCommChannelTechnosoft::getbaudrate(){return this->baudrate;}

int SerialCommChannelTechnosoft::init(int _hostID, const std::string& _pszDevName,const BYTE& _btType,const DWORD& _baudrate){
    //DPRINT("initializing dev %s type %d baud %d",_pszDevName.c_str(),_btType,_baudrate);
    pszDevName=_pszDevName;
    btType = _btType;
    baudrate = _baudrate;
    hostID = _hostID;
    fd = -1;
    return 0;
}

void SerialCommChannelTechnosoft::close(){
}

SerialCommChannelTechnosoft::~SerialCommChannelTechnosoft(){
    close();
    //DPRINT("Deallocazione oggetto SerialCommChannelTechnosoft. Invio comando di chiusura canale di comunicazione effettuato");
}

int SerialCommChannelTechnosoft::open(){

    return 0;
}


TechnoSoftLowDriver::TechnoSoftLowDriver(){
}

TechnoSoftLowDriver::~TechnoSoftLowDriver(){

    deallocateTimerAlarms = true;
    usleep(1500000);
    deinit();
}

double TechnoSoftLowDriver::speedfromMMsToIU(double _speed_mm_s){
    if (this->useUI)
	return _speed_mm_s; 
    return (N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360*_speed_mm_s) / CONVERSION_FACTOR_DEG_UI;
}

double TechnoSoftLowDriver::speedfromIUTOMMs(double _speed_IU){
    if (this->useUI)
        return _speed_IU; 
    return (_speed_IU*CONVERSION_FACTOR_DEG_UI)/(N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360);
}

double TechnoSoftLowDriver::accelerationfromMMs2ToIU(double _acceleration_mm_s2){
    if (this->useUI)
	return _acceleration_mm_s2;
    
    return (N_ROUNDS_DEFAULT/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT*360*_acceleration_mm_s2) / CONVERSION_FACTOR_DEGs2_UI;
}

double TechnoSoftLowDriver::accelerationfromIUToMMs2(double _acceleration_IU){
    if (this->useUI)
	return _acceleration_IU;
    
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
                        const double _range,  //[meter]
                        const double _fullScalePot, 
                        const int _alarmsPresent,
                        const double _alarmsInterval,
                        const double _percOfnoise,
                        const double _probabilityError
                        ){

   // DPRINT("Inizializzazione parametri");
    this->useUI=false;
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
    
    if(_alarmsInterval<=0)
        return -27;
    
    if(_percOfnoise<0.0 || _percOfnoise>1.0)
        return -28;

    percNoise = _percOfnoise;
    

    positiveLimitPosition = (long)((n_rounds*range)/linear_movement_per_n_rounds)*steps_per_rounds*const_mult_technsoft;
    

    std::srand(time(0)); // Inizializza generatore numeri pseudo-casuali
    
    
    if(_probabilityError<0 || _probabilityError>1)
        return -90;
        
    p = _probabilityError;

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX))
        return -28;


    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX))
        return -29;


    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX))
        return -30;

    /*	Execute the initialization of the drive (ENDINIT) */

    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX))
        return -31;

     // Settare il registro per la lettura dell'encoder

    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX))
        return -32;


    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX))
        return -33;

    readyState = true;
    internalHomingStateDefault=0;
    internalHomingStateHoming2=0;

    // Inizializzazione mutex

    // Inizializzazione parametri caratterizzanti la movimentazione

    // Motion parameters
    position = 0;
    positionCounter=0;
    positionEncoder=0;
    positionPotentiometer=0;
    actuatorIDInMotion = false;
    stopMotionCommand = false;
    powerOffCommand = false;
    absolutePosition=0;
    deltaPosition=0;
    cap_position=0;

    generator.seed(time(0)); // seed with the current time

    // ********* Info/Alarms request *********
    stateInfoRequest = false;
    alarmsInfoRequest = false;
    //-----
    regMERrequest = false;
    contentRegMER =0; // Fault driver register
    //-----
    regSRHrequest = false;
    contentRegSRH =0; // Partial Fault and state driver register
    //-----
    regSRLrequest = false;
    contentRegSRL = 0;

    // *********** Constants ************
    epsylon = 0;
    
    // ************ Limit transitions *************
    LNStransition = false;
    LPStransition = false;
    
    LSNactive=false; // BIT DI STATO
    LSPactive=false; // BIT DI STATO
    
    // ************ Thread manager *************
    motionscalled=0;
    
    controlledPositionHoming = false;
    homingStopped = false;
    
    deallocateTimerAlarms = false;
    deallocateTimerStates = false;
   // pthread_create(&thstaticFaultsGeneration, NULL,staticFaultsGeneration,this);
   // staticFaultsGeneration((void*)this);
    alarms=(bool)_alarmsPresent;
    durationAlarmsInterval=_alarmsInterval;
    
    controlLNS=true;
    return 0;
}

int TechnoSoftLowDriver::homing(int mode){
    // Attenzione: la variabile mode non viene utilizzata
    if(controlLNS){
        std::string descStr="";
        uint16_t contentRegMER=0;

        if(LSNactive){
            // Il LNS e' attivo. Non c'e' bisogno di effettuare la procedura di homing.
            if(selectAxis()<0){
                internalHomingStateDefault = 0;
                controlLNS=true;
                return -1;
            }

            if(resetEncoderHoming()<0){
                internalHomingStateDefault = 0;
                controlLNS=true; 
                cap_position=0;
                return -2;
            }
            

            
            if(resetCounterHoming()<0){
                internalHomingStateDefault = 0;
                controlLNS=true; 
                cap_position=0;
                return -3;   
            }
            
            cap_position=0;
            internalHomingStateDefault = 0;
            controlLNS=true;
            
            return 0;
        }
        // IL LNS non e' attivo. La procedura di homing deve cominciare, senza piu' fare questo controllo. 
        controlLNS=false;    
    }
       
    if(mode==0){
        //homingType = 0;
        int risp;
        int switchTransited=0;
        //int motionCompleted = 0;
        //std::string cappos = "CAPPOS";
        cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
        //int absoluteMotionCompleted = 0;

        switch (internalHomingStateDefault) {
            case 0:
                if(!homingStopped){
                    if(selectAxis()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true;
                        risp = -1;
                        break;
                    }
                    moveVelocityHoming(); // Nota: questo thread DOVRA ESSERE POI STOPPATO NEGLI STATI SUCCESSIVI DELLA FUNZIONE HOMING
                    //DPRINT(" STATE 0: move velocity activated ");
                    if(setEventOnLimitSwitch()<0){ // In pratica il setEventOnLimitSwitch non fa niente dal punto di vista funzionale nel caso virtuale
                        internalHomingStateDefault = 0;
                        controlLNS=true;
                        if(stopMotion()<0){
                            risp = -4;
                            break;
                        }
                        risp = -5;
                        break;
                    }
                    //DPRINT("STATE 0: event on limit switch activated ");
                    if(position>0){
                        internalHomingStateDefault = 1;
                    }
                    else{ // Andiamo direttamente a resettare encoder e counter
                        internalHomingStateDefault = 5;
                    }
                    risp = 1;
                    break;
                }
                else{
                    internalHomingStateDefault=6;
                    risp = 1;
                    break;
                } 
            case 1:
                if(!homingStopped){
                    if(selectAxis()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true;
                        risp = -6;
                        break;
                    }
                    usleep(3000);
                    if(checkEvent(switchTransited)<0){ // Se lo switch e' transitato si fermera' il motore (simulazione hardware)
                        internalHomingStateDefault = 0;// deve essere riinizializzato per successivi nuovi tentativi di homing
                        controlLNS=true;
                        if(stopMotion()<0){
                            risp = -7;
                            break;
                        }

                        risp = -8;
                        break;
                    }   
                    //DPRINT(" STATE 1: possible limit switch transition just checked ");
                    if(switchTransited){  // Lo switch e' transitato. La cap_position e' stata salvata. Il motore si sta per fermare
                        internalHomingStateDefault=2;
                    }
                // **************DA IMPLEMENTARE:***************** RESET ON Limit Switch Transition Event
                    risp = 1;
                    break;
                }
                else{
                    internalHomingStateDefault=6;
                    risp = 1;
                    break;
                }
            case 2:
                if(!homingStopped){
                    if(selectAxis()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true;
                        risp = -11;
                        break;
                    }

                    if(!actuatorIDInMotion){
                        // Il motore si e' fermato dopo che lo switch e' transitato
                        //DPRINT("************** STATE 2: Motion completed after transition **************");
                        internalHomingStateDefault = 3;
                    }
                    risp= 1;
                    //sleep(5);
                    break;
                }
                else{
                    internalHomingStateDefault=6;
                    risp = 1;
                    break;
                }
            case 3:
                if(!homingStopped){
                    // The motor is not in motion
                    //DPRINT("************** STATE 3: read the captured position on limit switch transition**************");
                    if(selectAxis()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true;
                        risp = -13;
                        break;
                    }
                    if(moveAbsoluteStepsHoming(cap_position)<0){ // ******************* motion per il recupero *********************
                        internalHomingStateDefault=0;
                        controlLNS=true;
                        risp = -15;
                        break;
                    }
                    //DPRINT("************** STATE 3: command of absolute positioning on the captured position sended **************");
                    internalHomingStateDefault = 4;
                    risp= 1;
                    //sleep(5);
                    break;
                }
                else{
                    internalHomingStateDefault=6;
                    risp = 1;
                    break;
                }
            case 4:
                //DPRINT("************** STATE 4:**************");
                if(!homingStopped){
                
                    //DPRINT("************** STATE 4: wait for positioning to end **************");
                    if(selectAxis()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true;
                        risp = -16;
                        break;
                    }
                    if(!actuatorIDInMotion){ // riposizionamento terminato, passa al prossimo stato
                    internalHomingStateDefault = 5;

                        DPRINT("************** STATE 4: motor positioned to end **************");
                    }
            // **************DA IMPLEMENTARE:*****************
            // RESET ON Event On Motion Complete
                //sleep(5)
                    risp= 1;
                    break;  
                }
                else{
                    internalHomingStateDefault=6;
                    risp = 1;
                    break;
                }
            case 5:
                //DPRINT("************** STATE 5:**************");
                if(!homingStopped){
                    // The motor is positioned to end
                    if(selectAxis()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true; 
                        risp = -19;
                        break;
                    }

                    if(resetEncoderHoming()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true; 
                        cap_position=0;
                        risp = -20;
                        break;
                    }
                    if(resetCounterHoming()<0){
                        internalHomingStateDefault = 0;
                        controlLNS=true; 
                        cap_position=0;
                        return -3;   
                    }  
                
                //DPRINT("************** STATE 5: encoder e counter e counter are reset **************");
                internalHomingStateDefault = 0;
                controlLNS=true;
                risp= 0;
                //sleep(5);
                break;
            }
            else{
                internalHomingStateDefault=6;
                risp = 1;
                break;
            }
            case 6:
                //DPRINT("************** STATE 6:**************");
                internalHomingStateDefault=0;
                controlLNS=true;
                homingStopped = false;
                risp = 0;
                break;
            default:
                internalHomingStateDefault = 0;
                controlLNS=true;
                risp= -24;
                //sleep(5);
                break;
        }
        return risp;
    }
    else if(mode==1){
        //homingType = 1;
        int switchTransited=0;
        int risp;
        switch (internalHomingStateHoming2) {
            case 0:
                if(!homingStopped){
                    //DPRINT("************** Homing procedure Homing2. STATE 0. **************");
                    if(moveVelocityHoming()<0){
                        internalHomingStateHoming2=0;
                        controlLNS=true;
                        risp= -1;
                        break;
                    }
                    if(position>0){
                        internalHomingStateHoming2=1;
                    }
                    else{
                        internalHomingStateHoming2=2; // Andiamo direttamente a resettare
                    }     
                    risp= 1;
                    break;
                }
                else{
                    internalHomingStateHoming2=3;
                    risp = 1;
                    break;
                }
            case 1:
                //DPRINT("************** Homing procedure Homing2. STATE 1. **************");

                if(!homingStopped){
                    usleep(2500);
                    if(checkEvent(switchTransited)<0){
                        //ERR("Reading state error: %s",descStr.c_str());
                        internalHomingStateHoming2=0;
                        controlLNS=true;

                        if(stopMotion()<0){
                            risp= -1;
                            break;
                        }

                        risp = -2;
                        break;
                    }
                    // lettura bit di interesse
                    //homingDone=((contentReg & ((uint16_t)1)<<7) != 0 ? 1 : 0);
                    if(switchTransited){// Lo switch e' transitato. La cap_position e' stata salvata. Il motore si sta per fermare
                        internalHomingStateHoming2=2;
                    }
                    risp= 1;
                    break;
                }
                else{
                    internalHomingStateHoming2=3;
                    risp = 1;
                    break;
                }
            case 2:
                //DPRINT("************** Homing procedure Homing2. STATE 2. **************");
                //DPRINT("************** Reset encoder e counter **************");
                if(!homingStopped){
                    
                    while(actuatorIDInMotion){
                        usleep(1000);
                    }
                    if(resetEncoderHoming()<0){
                        internalHomingStateHoming2 = 0;
                        controlLNS=true; 
                        cap_position=0;
                        risp = -3;
                        break;
                    }

                    if(resetCounterHoming()<0){
                        internalHomingStateHoming2 = 0;
                        controlLNS=true;
                        cap_position=0;
                        risp = -4;
                        break;  
                    }
                
                // Attendiamo che il motore si fermi prima di fare il reset:

                    internalHomingStateHoming2=0;
                    controlLNS=true;
                    risp=0;
                    break;
                }
                else{
                    internalHomingStateHoming2=3;
                    risp = 1;
                    break;
                }
            case 3:
                //DPRINT("************** STATE 6:**************");
                internalHomingStateHoming2=0;
                controlLNS=true;
                homingStopped = false;
                risp = 0;
                break;
            default:
                internalHomingStateHoming2=0;
                controlLNS=true;
                risp=-22;
                break;
        }
        return risp;
    }
    else{
        return -100;
    }
}

int TechnoSoftLowDriver::incrDecrPosition(){
	stopMotionCommand = false;
    bool goahead=false; // Per default vai indietro
	

    if(deltaPosition==0){
      
        return 0;
    }
    // In quale posizione mi devo spostare?
    if(deltaPosition>=0){
        goahead=true;
    }
	DPRINT("Setting state to motion incrDecr position of %ld",deltaPosition);
    actuatorIDInMotion = true;
   

    long initPosition = position;
    bool resetLimitSwicth=true;
	DPRINT("ALEDEBUG :ENTRIAMO NEL WHILE?");
	DPRINT("position %ld >= %d", position,(-10*SPEED_DEFAULT));
	DPRINT("position %ld <= %ld + %d", position,positiveLimitPosition,(10*SPEED_DEFAULT));
	DPRINT("abs(position-initPosition) %ld < abs(deltaPosition) %ld", abs(position - initPosition), abs(deltaPosition));
	DPRINT("stopMotionCommand %d", stopMotionCommand);
    while(position>=(-10*SPEED_DEFAULT) && (position<=positiveLimitPosition+10*SPEED_DEFAULT)  && abs(position-initPosition)<abs(deltaPosition) && !stopMotionCommand)
	{
		
        if(!powerOffCommand){
            

            if(goahead)
	    { // vai avanti
 		if (!this->useUI)
		{	
                	position+=speed_IU;
                	positionCounter+=speed_IU;
                	positionEncoder+=speed_IU;
                	positionPotentiometer+=speed_IU;
                	LNStransition = false;
                	LSNactive = false;
        	}
		else
		{
                	position+=1;
                	positionCounter+=1;
                	positionEncoder+=1;
                	positionPotentiometer+=1;
                	LNStransition = false;
                	LSNactive = false;
		}
            }
            else{ // vai indietro
 		if (!this->useUI)
		{	
                	position-=speed_IU;
                	positionCounter-=speed_IU;
                	positionEncoder-=speed_IU;
                	positionPotentiometer-=speed_IU;
                	LPStransition = false;
                	LSPactive=false;
		}
		else
		{	
                	position-=1;
                	positionCounter-=1;
                	positionEncoder-=1;
                	positionPotentiometer-=1;
                	LPStransition = false;
                	LSPactive=false;
		}
		
            }

            if(resetLimitSwicth)
	    {
                LSNactive=false;
                if (position<=positiveLimitPosition){
                    LSPactive=false;
                }    
                resetLimitSwicth=false;
            }
      
        }

        //DPRINT("Posizione incrementata!!!!!");
	if (!this->useUI)
        	usleep(1000); // Sleep for 1 milli second
	else
		usleep(100);
    }

    // DEBUG

        
    actuatorIDInMotion = false;
    stopMotionCommand = false;
    motionscalled--;


    
    if(position>positiveLimitPosition){
        LSPactive=true;
        positionCounter=positiveLimitPosition;
        positionEncoder=positiveLimitPosition;
        positionPotentiometer=positiveLimitPosition;
        position=positiveLimitPosition; 
    }
    
   
    
    return 0;
}

void* TechnoSoftLowDriver::staticIncrDecrPositionFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->incrDecrPosition();

    //DPRINT("Uscita dal thread di movimentazione");
   // pthread_exit(NULL);
   return NULL;
}


int TechnoSoftLowDriver::moveRelativeSteps(const long& _deltaPosition){

    //threadMoveRelativeOn=false; // Spegnamo il thread correntemente in esecuzione
    stopMotionCommand=false; 
    DPRINT("ALEDEBUG moving to %ld ",_deltaPosition)
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){

        
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
    motionscalled++;
    

    if(motionscalled>1){
        if(stopMotion()<0){
			DPRINT("ALEDEBUG exiting for stopMotion internal");
            return -2;
        }
        usleep(100000); // Attendi che la corrente movimentazione si fermi
    }

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
		DPRINT("ALEDEBUG exiting for fake Caschera Error");
        return -3;
    }

    deltaPosition = _deltaPosition;
    //cIP.ptr = this;
    
	//DPRINT("ALEDEBUG Launching staticIncrementDecrementPositionFunctionForThread");
  //  pthread_create(&th, NULL,TechnoSoftLowDriver::staticIncrDecrPositionFunctionForThread,this);
    TechnoSoftLowDriver::staticIncrDecrPositionFunctionForThread(this);
    return 0;
}


double TechnoSoftLowDriver::getdeltaMicroSteps(const double& deltaMillimeters){
    if (this->useUI)
    {
	DPRINT("ALEDEBUG conversion to be checked now %f",deltaMillimeters*const_mult_technsoft);
	//return deltaMillimeters*const_mult_technsoft;
	return deltaMillimeters;
    }

    return round((steps_per_rounds*n_rounds*const_mult_technsoft*deltaMillimeters)/linear_movement_per_n_rounds);
}


// Set trapezoidal parameters
int TechnoSoftLowDriver::setSpeed(const double& _speed_mm_s){
    
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

int TechnoSoftLowDriver::setRatiOfNoise(const double& _ratiOfNoise){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    //DPRINT("Chiamata setRatiOfNoise");
    if(_ratiOfNoise<0 || _ratiOfNoise>1){
        DERR("Ratio specified for noise = %f",_ratiOfNoise);
        return -1;
    }
    percNoise = _ratiOfNoise;
    return 0;
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

int TechnoSoftLowDriver::setMaxhighSpeedHoming(const double& _maxhighSpeedHoming_IU){

    if(_maxhighSpeedHoming_IU<=0){
        return -1;
    }
    
    double _maxHighSpeedHoming_IU = speedfromMMsToIU(_maxhighSpeedHoming_IU);
    
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
    positiveLimitPosition = (long)((n_rounds*range)/linear_movement_per_n_rounds)*steps_per_rounds*const_mult_technsoft;
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
    positiveLimitPosition = (long)((n_rounds*range)/linear_movement_per_n_rounds)*steps_per_rounds*const_mult_technsoft;
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
    positiveLimitPosition = (long)((n_rounds*range)/linear_movement_per_n_rounds)*steps_per_rounds*const_mult_technsoft;
    return 0;
}


int TechnoSoftLowDriver::setMeasureUnit(const bool& inSteps)
{
 this->useUI=inSteps;
 return 0;
}

int TechnoSoftLowDriver::getMeasureUnit(bool& inSteps)
{
 inSteps=this->useUI;
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
    positiveLimitPosition = (long)((n_rounds*range)/linear_movement_per_n_rounds)*steps_per_rounds*const_mult_technsoft;
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
    //positiveLimitPosition = (long)((n_rounds*range)/linear_movement_per_n_rounds)*steps_per_rounds*const_mult_technsoft;
    positiveLimitPosition = (long) 1000000000000000000;
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

int TechnoSoftLowDriver::setAlarmsGeneration(const int& _alarms){
    
    if(_alarms)
        alarms=true;
    else
        alarms=false;
    
    return 0;
}

int TechnoSoftLowDriver::getAlarmsGeneration(int& _alarms){
    
    _alarms=(int)alarms;
    
    return 0;
}

int TechnoSoftLowDriver::setAlarmsInterval(const double& _value){
    
    if(_value<0)
        return -1;
        
    durationAlarmsInterval=_value;
    
    return 0;
}

int TechnoSoftLowDriver::getAlarmsInterval(double& _alarms){
    
    _alarms=durationAlarmsInterval;
    
    return 0;
}

int TechnoSoftLowDriver::setPercOfNoise(const double& value){
    
    if(value<0.0 || value>1.0)
        return -1;
    
    percNoise=value;
    
    return 0;
}
    
int TechnoSoftLowDriver::getPercOfNoise(double& value){
    
    value=percNoise;
    return 0;
}

int TechnoSoftLowDriver::setProbError(const double& value){
    
    if(value<0.0 || value>1.0)
        return -1;
    
    p=value;
    
    return 0;
}

int TechnoSoftLowDriver::getProbError(double& value){
    
    value=p;
    return 0;
}

int TechnoSoftLowDriver::moveAbsolutePosition(){

    // Stima grossolana tempo necessario per la movimentazione

    bool goahead = false;
    stopMotionCommand=false;
    DPRINT("moveAbsolute starts here with stopMotionCommand=%d",stopMotionCommand);
   
    
    if(position<0){
       position=0; 
    }
    else if(position>positiveLimitPosition){
       position=positiveLimitPosition;  
    }
    
    bool turnLNS = false;
    bool turnLPS = false;
 
        //return -1;
    
    if(absolutePosition>positiveLimitPosition){
        absolutePosition=positiveLimitPosition;
        turnLPS=true;
    }
    
    if(position==(absolutePosition)){ //position: current position
   
        return 0;
    }
    
    // Quindi absolutePosition>=0 && absolutePosition <= LONG_MAX (perche' e' rappresentabile)
    // La slitta dunque si muovera' nella posizione [0,LONG_MAX]

    long initPosition = position; // mi prendo la posizione corrente del motorE
    actuatorIDInMotion = true;

    if(absolutePosition>initPosition){
        goahead = true; // Vai avanti
    }

    bool resetLimitSwicth = true;

    DPRINT("moveAbsolutePosition: appena prima del ciclo while");

    long tol = speed_IU;  //tolerance. Serve a una fava
    if (this->useUI)
	    tol=0;
   

    while(std::labs(position-absolutePosition)>tol && !stopMotionCommand){// L'incremento dovra' avvenire ad una determinata velocita'

        //DPRINT("moveAbsolutePosition: dentro il ciclo while: inizio prima del lock!!!");

        if(!powerOffCommand)
        {
          
            if(goahead)
	        { // vai avanti
                if (!this->useUI)
                {
                            position+=speed_IU;
                            positionCounter+=speed_IU;
                            positionEncoder+=speed_IU;
                            positionPotentiometer+=speed_IU;
                            LNStransition = false;
                            LSNactive = false;
                }
                else
                {  int nextStep= std::min((std::labs(position-absolutePosition)), (long) speed_IU) ;
                            position+=nextStep;
                            positionCounter+=nextStep;
                            positionEncoder+=nextStep;
                            positionPotentiometer+=nextStep;
                            LNStransition = false;
                            LSNactive = false;
                }
            }
            else
	    {
		if (!this->useUI)
		{
                	position-=speed_IU;
                	positionCounter-=speed_IU;
                	positionEncoder-=speed_IU;
                	positionPotentiometer-=speed_IU;
                	LPStransition = false;
                	LSPactive = false;
		}
		else
		{
                    int nextStep= std::min((std::labs(position-absolutePosition)), (long) speed_IU) ;
                	position-=nextStep;
                	positionCounter-=nextStep;
                	positionEncoder-=nextStep;
                	positionPotentiometer-=nextStep;
                	LPStransition = false;
                	LSPactive = false;
		}
            }

            if(resetLimitSwicth){
                if(position >= 0){
                    LSNactive=false;
                    resetLimitSwicth=false;
                }
                if(position<=positiveLimitPosition){
                    LSPactive=false;
                    resetLimitSwicth=false;
                }
            }

          
        }
	if (!this->useUI)
        	usleep(1000); // Sleep for 5 milli second
	else
		usleep(1000000);

    }
    //position=currentPosition;
  
    DPRINT("Putting actuatorIDInMotion(%d) to false. stopMotionCommand is %d",actuatorIDInMotion,stopMotionCommand);
    DPRINT("position is %ld, absolute is  %ld tolerance %ld",position,absolutePosition,tol); 
 
    actuatorIDInMotion = false;
    stopMotionCommand = false;
    motionscalled--;

    if(turnLNS && std::labs(position-absolutePosition)<=tol){ // nel qual caso absolutePosition dato in input ==0
        position =0;
        positionCounter=0;
        positionEncoder=0;
        positionPotentiometer=0;
        LSNactive = true;
    }
    else if(turnLPS && std::labs(position-absolutePosition)<=tol){ // nel qual caso absolutePosition dato in input ==0
        position = positiveLimitPosition; // non e' possibile assegnargli un valore piu grande :(
        positionCounter=positiveLimitPosition;
        positionEncoder=positiveLimitPosition;
        positionPotentiometer=positiveLimitPosition;
        LSPactive = true;
    }
     
  
    return 0;
}

void* TechnoSoftLowDriver::staticMoveAbsolutePositionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->moveAbsolutePosition();
    return NULL;
}

int TechnoSoftLowDriver::moveAbsoluteSteps(const long& absPosition){

    DPRINT("MoveAbsoluteSteps called with %ld",absPosition);
   
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -1;
//        }

        
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
    
    motionscalled++;
    
    if(motionscalled>1){
        stopMotion();
        usleep(100000); // Attendi che la corrente movimentazione si fermi
    }

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
       
        return -1;
    }

    absolutePosition=absPosition;

   
    //pthread_create(&th, NULL,staticMoveAbsolutePositionForThread,(void*)this);
    staticMoveAbsolutePositionForThread((void*)this);
    return 0;
}

int TechnoSoftLowDriver::moveConstantVelocityHoming(){

   

    // L'incremento deve avvenire ad una determinata velocita'
    if(position<=0){ // In questo caso posso gia' far finire con successo la procedura di homing
        //DPRINT("La posizione in steps e' gia' <= 0. Non occorre spostarsi ancora indietro per effettuare di nuovo l'homing");
       
//        if(!homingType){
//            internalHomingStateDefault = 5;
//        }
        return -1;
    }

    actuatorIDInMotion = true;
    
    // Mettiamoci al riparo anche in questo caso...
    LPStransition=false;
    LSPactive=false;

//    bool goahead = false;
//    if(highSpeedHoming_IU>=0)
//        goahead = true;
    stopMotionCommand = false;
    
    
    while(!stopMotionCommand && !LNStransition && !powerOffCommand){

        if(!powerOffCommand){
          
            position+=highSpeedHoming_IU;
            positionCounter+=highSpeedHoming_IU;
            positionEncoder+=highSpeedHoming_IU;
       
          
        }

        usleep(2300); // Sleep for 1 milli second
    }
 
        actuatorIDInMotion = false;
        if(stopMotionCommand){
            //DPRINT("Movimentazione all'indietro della procedura di homing bloccata: %ld",position);
            homingStopped=true;
            stopMotionCommand = false;
            //sleep(50); 
        }
        if(LNStransition){
            LSNactive = true;
        }
              
  

    return 0;
}

void* TechnoSoftLowDriver::staticMoveConstantVelocityHomingFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->moveConstantVelocityHoming();
    return NULL;
}

int TechnoSoftLowDriver::moveVelocityHoming(){
    
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }
    
    //pthread_create(&th, NULL,staticMoveConstantVelocityHomingFunctionForThread,this);
staticMoveConstantVelocityHomingFunctionForThread((void*)this);
    return 0;
}

int TechnoSoftLowDriver::moveAbsolutePositionHoming(){
    
    // Stima grossolana tempo necessario per la movimentazione
//    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_IU); //[s]
//    double tol = 30;
//    deltaT += (deltaT*tol/100);
//
//    double totalTimeInterval = 0;   // solo per far partire il ciclo while
//    struct timeval startTime,endTime;
//
//    gettimeofday(&startTime,NULL);


    //bool goahead = false;
  
    if(position==(absolutePosition)){ //position: current position

        //DPRINT("Recupero homing non necessario");
      
        return 0;
    }
 
    // Quindi absolutePosition>=0 && absolutePosition <= LONG_MAX (perche' e' rappresentabile)
    // La slitta dunque si muovera' nella posizione [0,LONG_MAX]

    long initPosition = position; // mi prendo la posizione corrente del motorE
    actuatorIDInMotion = true;

//    if(absolutePosition>initPosition){
//        goahead = true;
//    }

   
    //DPRINT("Posizione corrente dal quale partire: %ld",position);
    //DPRINT("Absolute position da raggiungere: %ld",absolutePosition); // In realtà absolutePosition è sempre zero

    long tol = 150;
    bool resetLimitSwicth=true;
    while(labs(position-absolutePosition)>tol && !stopMotionCommand){// L'incremento dovra' avvenire ad una determinata velocita'
        
        if(!powerOffCommand){
      

            //DPRINT("Go ahead = true");
            position+=lowSpeedHoming_IU;
            positionCounter+=lowSpeedHoming_IU;
            positionEncoder+=lowSpeedHoming_IU;
            positionPotentiometer+=lowSpeedHoming_IU;

            if(resetLimitSwicth){
                if (position >= 0){
                    LNStransition = false;
                    LSNactive=false;
                }
            if (position<=positiveLimitPosition){
                LPStransition = false;
                LSPactive=false;
            }
            resetLimitSwicth=false;
            }
        
        }

        usleep(1000); // Sleep for 1 milli second
    }
    //position=currentPosition;
  

    actuatorIDInMotion = false;
    stopMotionCommand = false;
    motionscalled--;

    return 0;
}

void* TechnoSoftLowDriver::staticMoveAbsolutePositionHomingFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->moveAbsolutePositionHoming();

    return NULL;
}

int TechnoSoftLowDriver::moveAbsoluteStepsHoming(const long& absPosition){

    //DPRINT("(homing) moving absolute steps. Axis: %d, absPosition %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,absPosition,speed_IU,acceleration_mm_s2,movement,referenceBase);

    // Simulazione dialogo con il drive/motor

   
    motionscalled++;

    if(motionscalled>1){
        stopMotion();
        usleep(100000); // Attendi che la corrente movimentazione si fermi
    }

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
      
        return -1;
    }

    absolutePosition=absPosition;
    

   // pthread_create(&th, NULL,staticMoveAbsolutePositionHomingFunctionForThread,this);
staticMoveAbsolutePositionHomingFunctionForThread((void*)this);
    return 0;
}

int TechnoSoftLowDriver::stopMotion(){

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }    

   

    stopMotionCommand = true;

    usleep(10000); // Aspettiamo che il thread che sta compiendo la movimentazione si fermi...
    
   
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(!TS_Stop()){
//            return -1;
//        }
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }

    return 0;
}

void* TechnoSoftLowDriver::staticStopMotionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->stopMotion();

return NULL;
}

int TechnoSoftLowDriver::hardreset(bool mode){
    
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }  
   
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
        
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
    
    if(mode){
//        if(!TS_Save()){
//            return -2;
//        }
    }

    // L'hardreset di fatto stacca l'alimentazione di fase
    powerOffCommand = true;
    usleep(500000); // Attendiamo che il motore che magari si stava muovendo si fermi
    positionCounter=0;
    positionEncoder=0;
    // Il potentiometro non viene resettato. 
    // Per riottenere la coerenza di questo con il coounter e l'encoder occorre fare l'homing 
    
    // HP: Gli allarmi presenti sul registro RegMer sono tutti risolvibili, 
    // Gli switch se erano alzati rimmarranno alzati dopo l'hardreset per cui:  
    contentRegMER=0;
  
    return 0;
}

int TechnoSoftLowDriver::providePower(){

    // Simulazione dialogo con il drive/motor
    
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }    
    
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
//        if(stopMotion()<0){

//            return -2;
//        }
//        usleep(10000);
        
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
    
    
    // *************** Comando *****************
    powerOffCommand = false;

//    /*	Wait for power stage to be enabled */
//    WORD sAxiOn_flag = 0;
//    while(sAxiOn_flag == 0){
//        /* Check the status of the power stage */
//        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
//
//            return -3;
//        }
//
//        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
//    }
    //poweron=true;
  
    return 0;
}

int TechnoSoftLowDriver::stopPower(){
    //DPRINT("stopping power to axis:%d",axisID);

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }
 
    
    powerOffCommand = true;
    
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){

        usleep(5000);
        
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
  

    return 0;
}

int TechnoSoftLowDriver::deinit(){ // Identical to TechnoSoftLowDriver::stopPower()
  //  pthread_join(thstaticFaultsGeneration,NULL);
   // pthread_join(th,NULL);

    //DPRINT("TechnoSoftLowDriver %d object is deallocated",axisID);
    return 0;
}

int TechnoSoftLowDriver::getCounter(double* deltaPosition_mm){
    
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }

    double pos = positionCounter;
    if (this->useUI)
       *deltaPosition_mm=pos;
    else
    *deltaPosition_mm = (pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
    return 0;
}

int TechnoSoftLowDriver::getEncoder(double* deltaPosition_mm){

    //DPRINT("Reading ENCODER position");
//    long aposition;
//    if(!TS_GetLongVariable("APOS", aposition)){
//        return -1;
//    }
    //DPRINT("E' questo il getEncoder???????????????");
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }

    double pos = positionEncoder;
    if(percNoise>0){
	double pos_mm;
    if (this->useUI)
    {
       //pos_mm=pos/const_mult_technsoft;
       pos_mm=pos;
    }
    else
       pos_mm = (pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
        if(pos_mm==0){
            pos_mm=0.000001;
        }
        double deltaNoise = (pos_mm*percNoise);
        //DPRINT("pos_mm=%f",pos_mm);
        //DPRINT("deltaNoise=%f",deltaNoise);
        const double rangeMin = pos_mm-deltaNoise;
        const double rangeMax = pos_mm+deltaNoise;
        //DPRINT("rangeMin=%f",rangeMin);
        //DPRINT("rangeMax=%f", rangeMax);
        NumberDistribution distribution(rangeMin, rangeMax);
        Generator numberGenerator(generator, distribution);
        *deltaPosition_mm = numberGenerator();
        //DPRINT("************** Position encoder of axisID 14: %4.13f **************",*deltaPosition_mm);
        //sleep(1);
    }
    else
    {
	if (this->useUI)
        {
           //*deltaPosition_mm=pos/const_mult_technsoft;
           *deltaPosition_mm=pos;
        }
        else
           *deltaPosition_mm=(pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
    } 
    //DPRINT("Real position encoder %f millimeter", *deltaPosition_mm);
   
    return 0;
}

int TechnoSoftLowDriver::getPotentiometer(double* deltaPosition_mm){

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }

    double pos = position;
    
    if(percNoise>0){
        double pos_mm =(!this->useUI)? (pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft) : pos;
        if(pos_mm==0){
            pos_mm=0.000001;
        }
        double deltaNoise = (pos_mm*percNoise);
        //DPRINT("pos_mm=%f",pos_mm);
        //DPRINT("deltaNoise=%f",deltaNoise);
        const double rangeMin = pos_mm-deltaNoise;
        const double rangeMax = pos_mm+deltaNoise;
        //DPRINT("rangeMin=%f",rangeMin);
        //DPRINT("rangeMax=%f", rangeMax);
        NumberDistribution distribution(rangeMin, rangeMax);
        Generator numberGenerator(generator, distribution);
        *deltaPosition_mm = numberGenerator();
        //DPRINT("************** Position encoder of axisID 14: %4.13f **************",*deltaPosition_mm);
        //sleep(1);
    }
    else
    {
	if (this->useUI)
         *deltaPosition_mm=pos;
        else
         *deltaPosition_mm=(pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
   } 
    //DPRINT("Real position encoder %f millimeter", *deltaPosition_mm);
   
    return 0;
} 

int TechnoSoftLowDriver::getLVariable(std::string& nameVar, long& var) {


    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }
  

    var = position;
    
   

    return 0;
}

int TechnoSoftLowDriver::resetCounterHoming(){

//    int random_variable = std::rand();
//    if(random_variable<p*(RAND_MAX/100)){
//        return -1;
//    }
    
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }
  
    positionCounter = 0;

    return 0;
}

int TechnoSoftLowDriver::resetEncoderHoming(){
    
    // Simulazione dialogo con il drive/motor
  
//    
//    pthread_t th;
//    pthread_create(&th, NULL,staticResetEncoderForThread,this);
    sleep(3); // wait 3 seconds in order to simulate a movement
    positionEncoder = 0;
    //position = 0;
   
    return 0;
}

void* TechnoSoftLowDriver::staticResetEncoderForThread(void* objPointer){

    ((TechnoSoftLowDriver*)objPointer)->resetEncoderHoming();
    return NULL;
}

int TechnoSoftLowDriver::setEventOnLimitSwitch(short lswType, short transitionType, BOOL waitEvent, BOOL enableStop){

    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)) {
        return -1;
    }

    return 0;
}

int TechnoSoftLowDriver::setEventOnMotionComplete(BOOL waitEvent, BOOL enableStop){
    
    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)) {
        return -1;
    }

    return 0;
}


int TechnoSoftLowDriver::checkEvent(BOOL& event){

    //Evento che segnala il limit switch transition

    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)) {
        return -1;
    }
    
//    if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//    }
    
        if(position <= -epsylon){
            //DPRINT("Rilevata posizione <= 0: %ld", position);
            LNStransition = true;// Occorre "bloccare" il motion, ovvero terminare il thread che si sta occupando del motion
            cap_position = position;
            event = true;        // comunichiamo alla funzione homing l'avvenuta transizione del LSN
        //sleep(30);              
        //stopMotionCommand=true; //Serve a far terminare la movimentazione  
        }
        else{
            //DPRINT("Posizione riscontrata durante il checking: %ld", position);
            LNStransition = false;
            event = false;
        //sleep(30);
        }
   
    return 0;
}

int TechnoSoftLowDriver::getStatusOrErrorReg(const short& regIndex, WORD& contentRegister, std::string& descrErr){
    
    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)) {
        //descrErr=descrErr+" Error reading status: "+TS_GetLastErrorText();
        return -1;
    }
    
  
    
    if(alarmsInfoRequest && regMERrequest){
        contentRegister=contentRegMER;
     
        return 0;
    }

    else if(alarmsInfoRequest && regSRHrequest){
        contentRegister=contentRegSRH;
      
        return 0;
    }
  
    else if(stateInfoRequest && regSRHrequest){
        contentRegister=0;
   
        return 0;
    }

    else if(stateInfoRequest && regSRLrequest){
        contentRegister = 0;
 
        return 0;
    }
    
    return -1;
}

int TechnoSoftLowDriver::faultsGeneration(){
    
    struct timeval startTimeForMotor1,endTimeForMotor1;
    
    double total_time_interval=0;
    
    WORD contatoreRegMer = 0;
    WORD maxcontatoreRegMer = 15;
    const double rangeMinRegMer = 0;
    const double rangeMaxRegMer = 14; // e non 15 perche' 15 individua il bit dell'emergency. 
    
//    WORD contatoreRegSRH = 0;
//    WORD maxcontatoreRegSRH = 5;
//    const double rangeMinRegSRH = 10;
//    const double rangeMaxRegSRH = 11;
    
    WORD positionRegMer = 6;
    int numTentativi=0;
    int maxNumTentativi=3;
//    WORD positionRegSRH = 0;

    gettimeofday(&startTimeForMotor1,NULL);
    while(!deallocateTimerAlarms){
 
        // Lettura ogni secondo...
        if(total_time_interval>durationAlarmsInterval){
            
            if(alarms){
 
                contentRegMER = 0;
                while((numTentativi<maxNumTentativi) && (positionRegMer==6 || positionRegMer==7)){
                    NumberDistribution distribution(rangeMinRegMer, rangeMaxRegMer);
                    Generator numberGenerator(generator, distribution);
                    positionRegMer = (WORD)numberGenerator();
                    numTentativi++;
                }
                contentRegMER |= ((WORD)1<<positionRegMer);
                //contatoreRegMer++;
                numTentativi=0;
                positionRegMer=6;
 
 
            }
            gettimeofday(&startTimeForMotor1,NULL);
        }
        
        gettimeofday(&endTimeForMotor1,NULL);
        total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);

        usleep(1000000); 
    }
    return 0;
}

void* TechnoSoftLowDriver::staticFaultsGeneration(void* objPointer){
    
    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->faultsGeneration();

    //DPRINT("Uscita dal thread faultsGeneration");
    return NULL;
}

int TechnoSoftLowDriver::resetFault(){

    //DPRINT("LSNactive resettato prima del lock");
   
    if(internalHomingStateHoming2 != 0 || internalHomingStateDefault !=0){
        internalHomingStateHoming2 = 0;
        internalHomingStateDefault = 0;
        controlLNS=true;
    }
    
    //Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
   
        return -2;
    }
    
    contentRegMER = 0;
    //contentRegSRH = 0;
        // ma nn solo...
//    LSPactive = false;
//    LSNactive = false;
    
    //DPRINT("LSNactive resettato dentro il lock");
   
       
    return 0;
}

int TechnoSoftLowDriver::selectAxis(){

    //Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }    

    return 0;
}

// retrieve firmware version of the active drive
int TechnoSoftLowDriver::getFirmwareVers(char* firmwareVers){

    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX)){
        return -1;
    }
    char msg[] = "Simulated firmware.";
    firmwareVers = msg;

    return 0;
}
