#include "TechnoSoftLowDriver.h"
#include <common/debug/core/debug.h>
#include <iostream>

using namespace common::actuators::models::simul;

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

//    if(fd!=-1){
//	TS_CloseChannel(fd); // chiusura canale di comunicazione
//    }
    DPRINT("Chiusura canale di comunicazione in fase di deallocazione delle risorse");
}

SerialCommChannelTechnosoft::~SerialCommChannelTechnosoft(){
    close();
    DPRINT("Deallocazione oggetto SerialCommChannelTechnosoft. Invio comando di chiusura canale di comunicazione effettuato");
}

int SerialCommChannelTechnosoft::open(){
//    int resp;
//    /*	Open the comunication channel: COM1, RS232, 1, 115200 */
//    DPRINT("opening dev %s type %d baud %d, hostid:%d",pszDevName.c_str(),btType,baudrate,hostID);
//    resp=TS_OpenChannel(pszDevName.c_str(), btType, hostID, baudrate);
//    if(resp < 0){
//      ERR("failed opening channel dev:%s type:%d host:%d baudrate: %d",pszDevName.c_str(), btType, hostID, baudrate);
//      return -1;
//    }
//    this->fd = resp;
//    DPRINT("Openchannel file descriptor=%d",resp);
//    std::srand(std::time(0));
//    random_variable = std::rand();
//    if(random_variable<p*(RAND_MAX/100))
//        return -1;
    
    DPRINT("Channel is opened");
    return 0;
}

//TechnoSoftLowDriver::channel_map_t TechnoSoftLowDriver::channels; // Anche se non viene inizializzato...

//----------------------------------------------
TechnoSoftLowDriver::TechnoSoftLowDriver(){

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
                        const double _linear_movement_per_n_rounds,
                        const double _percOfNoise){

    DPRINT("Inizializzazione parametri");

    if(_maxSpeed_mm_s<=0){
        return -1;
    }
    maxSpeed_mm_s=_maxSpeed_mm_s;
    //DPRINT("maxSpeed_mm_s = %f",maxSpeed_mm_s);

    if(_speed_mm_s<=0 || _speed_mm_s>_maxSpeed_mm_s){
        return -2;
    }
    speed_ms_s = _speed_mm_s;
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
    
    if(_percOfNoise<0 || _percOfNoise>1){
        return -23;
    }
    percNoise = _percOfNoise;

//    axisRef = TS_LoadSetup(setupFilePath.c_str());
//    if(axisRef < 0){
//        DERR("LoadSetup failed \"%s\"",setupFilePath.c_str());
//        return -24;
//    }
    std::srand(std::time(0)); // Inizializza generatore numeri pseudo-casuali
    p = 0.0;

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -24;

    /*	Setup the axis based on the setup data previously, for axisID*/
//    if(!TS_SetupAxis(_axisID, axisRef)){
//        DERR("failed to setup axis %d, %s",axisID,TS_GetLastErrorText());
//        return -25;
//    }
    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -25;

//    if(!TS_SelectAxis(_axisID)){
//        DERR("failed to select axis %d, %s",_axisID,TS_GetLastErrorText());
//        return -26;
//    }
    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -26;

    /*	Execute the initialization of the drive (ENDINIT) */
//    if(!TS_DriveInitialisation()){
//        DERR("failed Low driver initialisation");
//        return -27;
//    }
    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -27;

     // Settare il registro per la lettura dell'encoder
//    if(!TS_Execute("SCR=0x4338")){
//        //descrErr=descrErr+" "+TS_GetLastErrorText()+". ";
//        DERR("Failed TS_Execute command");
//        return -28;
//    }
    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -28;

//    if(!TS_SetEventOnMotionComplete(0,0)){
//	return -30;
//    }
    random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -29;

    readyState = true;
    internalHomingStateDefault=0;
    internalHomingStateHoming2=0;

    // Inizializzazione mutex
    pthread_mutex_init(&(mu),NULL);

    // Inizializzazione parametri caratterizzanti la movimentazione

    // Motion parameters
    position = 0;
    positionCounter=0;
    positionEncoder=0;
    actuatorIDInMotion = false;
    stopMotionCommand = false;
    powerOffCommand = false;
    absolutePosition=0;
    deltaPosition=0;
    cap_position=0;

    generator.seed(std::time(0)); // seed with the current time

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

//    // State of possible threads
//    threadMoveRelativeOn = false;
//    threadMoveAbsoluteOn = false;

    // ************ Limit transitions *************
    LNStransition = false;
    LSNactive=false; // BIT DI STATO
    LSPactive=false; // BIT DI STATO

    // ************ Thread manager *************
    motionscalled=0;
    
    //deltaNoise = 100; // Number of microsteps. Used for noise

    return 0;
}

int TechnoSoftLowDriver::homing(int mode){
    // Attenzione: la variabile mode non viene utilizzata

    if(mode==0){

        int risp;
        int switchTransited=0;
        int motionCompleted = 0;
        std::string cappos = "CAPPOS";
        cap_position = 0; /* the position captures at HIGH-LOW transition of negative limit switch */
        int absoluteMotionCompleted = 0;

        switch (internalHomingStateDefault) {
            case 0:
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -1;
                    break;
                }
//                if(moveVelocityHoming()<0){ // Nota: questo thread DOVRA ESSERE POI STOPPATO NEGLI STATI SUCCESSIVI DELLA FUNZIONE HOMING
//                    internalHomingStateDefault = 0;
////                    if(stopMotion()<0){
////                        risp = -2;
////                        break;
////                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this); // Nota: questo thread DOVRA ESSERE STOPPATO
//                    pthread_join(th,NULL);
//                    risp = -3;
//                    break;
//                }
                moveVelocityHoming(); // Nota: questo thread DOVRA ESSERE POI STOPPATO NEGLI STATI SUCCESSIVI DELLA FUNZIONE HOMING
                DPRINT(" STATE 0: move velocity activated ");
                if(setEventOnLimitSwitch()<0){ // In pratica il setEventOnLimitSwitch non fa niente dal punto di vista funzionale nel caso virtuale
                    internalHomingStateDefault = 0;
                    if(stopMotion()<0){
                        risp = -4;
                        break;
                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
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
                if(checkEvent(switchTransited)<0){ // Se lo switch e' transitato si fermera' il motore (simulazione hardware)
                    internalHomingStateDefault = 0;// deve essere riinizializzato per successivi nuovi tentativi di homing
                    if(stopMotion()<0){
                        risp = -7;
                        break;
                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
                    risp = -8;
                    break;
                }
                DPRINT(" STATE 1: possible limit switch transition just checked ");
                if(switchTransited){  // Lo switch e' transitato. La cap_position e' stata salvata. Il motore si sta per fermare
//                    if(setEventOnMotionComplete()<0){
//                        internalHomingStateDefault = 0; // deve essere riinizializzato per successive operazione di homing
////                        if(stopMotion()<0){
////                            risp= -9;
////                            break;
////                        }
//                        pthread_t th;
//                        pthread_create(&th, NULL,staticStopMotionForThread,this);
//                        pthread_join(th,NULL);
//                        risp =-10;
//                        break;
//                    }
                    //eventOnMotionCompleteSet = true;
                    internalHomingStateDefault=2;
                    DPRINT(" STATE 1: Negative limit switch transited. Event on motion completed set ");
                    DPRINT("Captured position: %ld",cap_position);
                }
                //sleep(5);
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
//                if(checkEvent(motionCompleted)<0){
//                    internalHomingStateDefault = 0;
//                    if(stopMotion()<0){
//                        risp = -12;
//                        break;
//                    }
//                    risp= -13;
//                    break;
//                }
//                if(position<=-epsylon)
//                    motionCompleted = true;
                DPRINT("************** STATE 2: possible event on motion completed checked **************");
//                if(motionCompleted){
//                    DPRINT("************** STATE 2: Motion completed after transition **************");
//                    internalHomingStateDefault = 3;
//                }
                if(!actuatorIDInMotion){
                    // Il motore si e' fermato dopo che lo switch e' transitato
                    DPRINT("************** STATE 2: Motion completed after transition **************");
                    internalHomingStateDefault = 3;
                }
                risp= 1;
                //sleep(5);
                break;
            case 3:
                // The motor is not in motion
                //DPRINT("************** STATE 3: read the captured position on limit switch transition**************");
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -13;
                    break;
                }
//                if(getLVariable(cappos, cap_position)<0){
//    //                if(stopMotion()<0){
//    //                    return -13;
//    //                }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
//                    internalHomingStateDefault=0;
//                    risp = -14;
//                    break;
//                }
                DPRINT("************** STATE 3: the captured position on limit switch transition is %ld [drive internal position units]**************",cap_position);

            /*	Command an absolute positioning on the captured position */
                if(moveAbsoluteStepsHoming(cap_position)<0){ // ******************* motion per il recupero *********************
                    internalHomingStateDefault=0;
                    risp = -15;
                    break;
                }
                DPRINT("************** STATE 3: command of absolute positioning on the captured position sended **************");
                internalHomingStateDefault = 4;
                risp= 1;
                //sleep(5);
                break;
            case 4:
                //DPRINT("************** STATE 4: wait for positioning to end **************");
                if(selectAxis()<0){
                    internalHomingStateDefault = 0;
                    risp = -16;
                    break;
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
                if(!actuatorIDInMotion){ // riposizionamento terminato, passa al prossimo stato
                    internalHomingStateDefault = 5;
                    DPRINT("************** STATE 4: motor positioned to end **************");
                }
            // **************DA IMPLEMENTARE:*****************
            // RESET ON Event On Motion Complete
                //sleep(5);
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
                    if(stopMotion()<0){
                        risp = -20; 
                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
                    risp= -21;
                    break;
                }
//                pthread_t th1;
//                pthread_create(&th1, NULL,staticResetEncoderForThread,this);
//                pthread_join(th1,NULL);
                if(resetCounter()<0){
                    internalHomingStateDefault = 0;
                    if(stopMotion()<0){
                    risp= -22;
                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
                    risp= -23;
                    break;
                }
//                pthread_t th2;
//                pthread_create(&th2, NULL,staticResetCounterForThread,this);
//                pthread_join(th2,NULL);
                DPRINT("************** STATE 5: encoder e counter e counter are reset **************");
                internalHomingStateDefault = 0;
                risp= 0;
                //sleep(5);
                break;
            default:
                internalHomingStateDefault = 0;
                risp= -24;
                //sleep(5);
                break;
        }
        return risp;
    }
    else if(mode==1){
        int switchTransited=0;
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
//                if((getStatusOrErrorReg(5, contentReg, descStr))<0){
//                    //ERR("Reading state error: %s",descStr.c_str());
//                    internalHomingStateHoming2=0;
//                    if(stopMotion()<0){
//                        risp= -2;
//                        break;
//                    }
//                    risp= -3;
//                    break;
//                }
                if(checkEvent(switchTransited)<0){
                    //ERR("Reading state error: %s",descStr.c_str());
                    internalHomingStateHoming2=0;
//                    if(stopMotion()<0){
//                        risp= -2;
//                        break;
//                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
                    if(stopMotion()<0){
                        risp= -1;
                        break;
                    }
//                    pthread_t th;
//                    pthread_create(&th, NULL,staticStopMotionForThread,this);
//                    pthread_join(th,NULL);
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
            case 2:
                DPRINT("************** Homing procedure Homing2. STATE 2. **************");
                DPRINT("************** Reset encoder e counter **************");
                
                while(actuatorIDInMotion){
                    usleep(1000);
                }
                
                if(resetEncoder()<0){
                    internalHomingStateHoming2=0;
                    if(stopMotion()<0){
                        risp= -1;
                        break;
                    }
                    risp= -2;
                    break;
                }
                
                if(resetCounter()<0){
                    internalHomingStateHoming2=0;
                    if(stopMotion()<0){
                        risp= -3;
                        break;
                    }
                    risp= -4;
                    break;
                }

                // Attendiamo che il motore si fermi prima di fare il reset:

//                while(actuatorIDInMotion){
//                    usleep(1000);
//                }

                // Il motore e' fermo. Possiamo fare il reset
//                pthread_t th1;
//                pthread_create(&th1, NULL,staticResetEncoderForThread,this);
//                pthread_join(th1,NULL);
                
//                if(resetCounter()<0){
//                    internalHomingStateHoming2=0;
//                    if(stopMotion()<0){
//                        risp= -1;
//                        break;
//                    }
//                    risp= -2;
//                    break;
//                }
                
//                if(resetEncoder()<0){
//                    internalHomingStateHoming2=0;
//                    if(stopMotion()<0){
//                        risp= -3;
//                        break;
//                    }
//                    risp= -4;
//                    break;
//                }
//                pthread_t th2;
//                pthread_create(&th2, NULL,staticResetCounterForThread,this);
//                pthread_join(th2,NULL);
//                if(resetCounter()<0){
//                    internalHomingStateHoming2=0;
//                    if(stopMotion()<0){
//                        risp= -6;
//                        break;
//                    }
//                    risp= -7;
//                    break;
//                }
                internalHomingStateHoming2=0;
                risp=0;
                break;
            default:
                internalHomingStateHoming2=0;
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

    // Stima grossolana tempo necessario per la movimentazione
//    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//    double tol = 30;
//    deltaT += (deltaT*tol/100);
//
//    double totalTimeInterval = 0;   // solo per far partire il ciclo while
//    struct timeval startTime,endTime;
//    gettimeofday(&startTime,NULL);

//    while(totalTimeInterval<=deltaT && !stopMotionCommand && !powerOffCommand){
//        actuatorIDInMotion = true;
//        // L'incremento deve avvenire ad una determinata velocita'
//        if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//        }
//            if(((containerIncrementPosition*)arg)->deltaPosition>=0){
//                position+=speed_ms_s;
//            }
//            else{
//                position-=speed_ms_s;
//            }
//        if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//        }
//        // Aggiornamento deltaT, stopMotion, stopPower
//        deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//        deltaT += (deltaT*tol/100);
//
//        gettimeofday(&endTime,NULL);
//        totalTimeInterval = ((double)endTime.tv_sec+(double)endTime.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
//        sleep(1); // Sleep for 1 second
//    }

    // L'incremento deve avvenire ad una determinata velocita'
//    if(pthread_mutex_lock(&(cIP.mu))!=0){
//
//    }
//    //threadMoveRelativeOn = true;
//    stopMotionCommand = false;
//    actuatorIDInMotion = true; //deltaPosition>=0
//    if(pthread_mutex_unlock(&(cIP.mu))!=0){
//
//    }
//
//    long initPosition = position;
//    while(fabs(position-initPosition)<=fabs(cIP.deltaPosition) && !stopMotionCommand && !powerOffCommand){
//
//        if(pthread_mutex_lock(&(cIP.mu))!=0){
//
//        }
//        if(cIP.deltaPosition>=0){
//            position+=speed_ms_s;
//            positionCounter+=speed_ms_s;
//            positionEncoder+=speed_ms_s;
//        }
//        else{
//            position-=speed_ms_s;
//            positionCounter-=speed_ms_s;
//            positionEncoder-=speed_ms_s;
//        }
//        if(pthread_mutex_unlock(&(cIP.mu))!=0){
//
//        }
//        usleep(1000); // Sleep for 1 milli second
//    }
//    //position = currentPosition;
//
//    if(pthread_mutex_lock(&(cIP.mu))!=0){
//
//    }
//    actuatorIDInMotion = false;
//    stopMotionCommand = false;
//
//    if(pthread_mutex_unlock(&(cIP.mu))!=0){
//
//    }
//    pthread_exit(NULL);
//
//    return 0;

    //DPRINT("Thread di movimentazione partito!!!!!!!!!");

    bool goahead=false;
    if(pthread_mutex_lock(&(mu))!=0){

    }

    if(deltaPosition==0){
        if(pthread_mutex_unlock(&(mu))!=0){
        }
        return -1;
    }
    // In quale posizione mi devo spostare?
    if(deltaPosition>=0){
        goahead=true;
    }

    actuatorIDInMotion = true;
    if(pthread_mutex_unlock(&(mu))!=0){

    }

    long initPosition = position;
    bool resetLimitSwicth=true;

    //DPRINT("%ld",position);

//    if(!(position>=0)){
//        DPRINT("!(position>=0)");
//    }
//
//    if(!(position<LONG_MAX)){
//        DPRINT("!(position<LONG_MAX)");
//    }
//
//    if(!(abs(position-initPosition)<=abs(deltaPosition)))
//        DPRINT("!(abs(position-initPosition)<=abs(deltaPosition))");
//
//    if(stopMotionCommand)
//        DPRINT("stopMotionCommand");
//
//    if(powerOffCommand)
//        DPRINT("powerOffCommand");

    while(position>=(-50*SPEED_DEFAULT) && position<=LONG_MAX && abs(position-initPosition)<=abs(deltaPosition) && !stopMotionCommand && !powerOffCommand){

        if(pthread_mutex_lock(&(mu))!=0){

        }

        if(goahead){
            position+=speed_ms_s;
            positionCounter+=speed_ms_s;
            positionEncoder+=speed_ms_s;
        }
        else{
            position-=speed_ms_s;
            positionCounter-=speed_ms_s;
            positionEncoder-=speed_ms_s;
        }

        //DPRINT("Posizione  dopo l'incremento effettuato: %ld", position);
        //DPRINT("Posizione encoder dopo l'incremento effettuato: %ld", positionEncoder);
        //DPRINT("Posizione counter dopo l'incremento effettuato: %ld", positionCounter);

        if(resetLimitSwicth){
            if (position >= 0){
                LSNactive=false;
            }
            if (position<=LONG_MAX){
                LSPactive=false;
            }
            resetLimitSwicth=false;
        }

        if(pthread_mutex_unlock(&(mu))!=0){

        }
        //DPRINT("Posizione incrementata!!!!!");
        usleep(1000); // Sleep for 1 milli second
    }

    // DEBUG
//    if(!(position>=0)){
//        DPRINT("!(position>=0)");
//    }
//
//    if(!(position<LONG_MAX)){
//        DPRINT("!(position<LONG_MAX)");
//    }
//
//    if(!(abs(position-initPosition)<=abs(deltaPosition)))
//        DPRINT("!(abs(position-initPosition)<=abs(deltaPosition))");
//
//    if(stopMotionCommand)
//        DPRINT("stopMotionCommand");
//
//    if(powerOffCommand)
//        DPRINT("powerOffCommand");

    if(pthread_mutex_lock(&(mu))!=0){

    }
        
    actuatorIDInMotion = false;
    stopMotionCommand = false;
    motionscalled--;

//        if(position==0){ // nel qual caso absolutePosition dato in input ==0
//            LSNactive = true;
//        }
    if(position<0){
        LSNactive=true;                            
//        position=0;
//        positionCounter=0;
//        positionEncoder=0;
    }
    
    if(position>LONG_MAX){  // nel qual caso absolutePosition dato in input == LONG_MAX
        LSPactive=true;
    }
    
    if(pthread_mutex_unlock(&(mu))!=0){

    }
    return 0;
}

void* TechnoSoftLowDriver::staticIncrDecrPositionFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->incrDecrPosition();

    DPRINT("Uscita dal thread di movimentazione");
    pthread_exit(NULL);
}

int TechnoSoftLowDriver::moveRelativeSteps(const long& _deltaPosition){

    DPRINT("Relative Moving axis: %d, deltaMicroSteps %l, speed=%f, acceleration %f, isadditive %d, movement %d, referencebase %d",axisID,deltaPosition,speed_ms_s,acceleration_mm_s2,isAdditive,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveRelative(deltaPosition, speed_mm_s, acceleration_mm_s2, isAdditive, movement, referenceBase)){
//        DERR("error relative moving");
//        return -2;
//    }

    //threadMoveRelativeOn=false; // Spegnamo il thread correntemente in esecuzione

    if(pthread_mutex_lock(&(mu))!=0){

    }
    motionscalled++;
    if(pthread_mutex_unlock(&(mu))!=0){

    }

    if(motionscalled>1){
        stopMotion();
        usleep(100000); // Attendi che la corrente movimentazione si fermi
    }

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/20))
        return -1;

    pthread_t th;
    deltaPosition = _deltaPosition;
    //cIP.ptr = this;

    pthread_create(&th, NULL,TechnoSoftLowDriver::staticIncrDecrPositionFunctionForThread,this);

    return 0;
}

//int TechnoSoftLowDriver::incrDecrPositionHoming(){
//
//    // Stima grossolana tempo necessario per la movimentazione
////    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
////    double tol = 30;
////    deltaT += (deltaT*tol/100);
////
////    double totalTimeInterval = 0;   // solo per far partire il ciclo while
////    struct timeval startTime,endTime;
////    gettimeofday(&startTime,NULL);
//
////    while(totalTimeInterval<=deltaT && !stopMotionCommand && !powerOffCommand){
////        actuatorIDInMotion = true;
////        // L'incremento deve avvenire ad una determinata velocita'
////        if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
////
////        }
////            if(((containerIncrementPosition*)arg)->deltaPosition>=0){
////                position+=speed_ms_s;
////            }
////            else{
////                position-=speed_ms_s;
////            }
////        if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
////
////        }
////        // Aggiornamento deltaT, stopMotion, stopPower
////        deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
////        deltaT += (deltaT*tol/100);
////
////        gettimeofday(&endTime,NULL);
////        totalTimeInterval = ((double)endTime.tv_sec+(double)endTime.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
////        sleep(1); // Sleep for 1 second
////    }
//
//    // L'incremento deve avvenire ad una determinata velocita'
//    if(pthread_mutex_lock(&(cIP.mu))!=0){
//
//    }
//    stopMotionCommand = false;
//    actuatorIDInMotion = true; //deltaPosition>=0
//    long currentPosition = position;
//    while((fabs(currentPosition-position)<=fabs(cIP.deltaPosition) && !stopMotionCommand && !powerOffCommand)){
//
//        if(cIP.deltaPosition>=0){
//            currentPosition+=speed_ms_s;
//        }
//        else{
//            currentPosition-=speed_ms_s;
//        }
//        usleep(1000); // Sleep for 1 milli second
//    }
//    position = currentPosition;
//    actuatorIDInMotion = false;
//    stopMotionCommand = false;
//
//    if(pthread_mutex_unlock(&(cIP.mu))!=0){
//
//    }
//    pthread_exit(NULL);
//}

double TechnoSoftLowDriver::getdeltaMicroSteps(const double& deltaMillimeters){

    return round((steps_per_rounds*n_rounds*const_mult_technsoft*deltaMillimeters)/linear_movement_per_n_rounds);
}

//void* TechnoSoftLowDriver::staticIncrDecrPositionHomingFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads
//
//    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
//    ((TechnoSoftLowDriver*)objPointer)->incrDecrPositionHoming();
//
//    return 0;
//}




//int TechnoSoftLowDriver::moveRelativeStepsHoming(const long& deltaPosition){
//    DPRINT("Relative Moving axis: %d, deltaMicroSteps %d, speed=%f, acceleration %f, isadditive %d, movement %d, referencebase %d",axisID,deltaPosition,highSpeedHoming_mm_s,accelerationHoming_mm_s2,isAdditiveHoming,movementHoming,referenceBaseHoming);
//
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -1;
////    }
////    if(!TS_MoveRelative(deltaPosition, highSpeedHoming_mm_s, accelerationHoming_mm_s2, isAdditiveHoming, movementHoming, referenceBaseHoming)){
////        DERR("error relative moving homing");
////        return -2;
////    }
//    DPRINT("Relative Moving axis: %d, deltaMicroSteps %d, speed=%f, acceleration %f, isadditive %d, movement %d, referencebase %d",axisID,deltaPosition,speed_ms_s,acceleration_mm_s2,isAdditive,movement,referenceBase);
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -1;
////    }
////    if(!TS_MoveRelative(deltaPosition, speed_mm_s, acceleration_mm_s2, isAdditive, movement, referenceBase)){
////        DERR("error relative moving");
////        return -2;
////    }
//    int random_variable = std::rand();
//    if(random_variable<p*(RAND_MAX/20))
//        return -1;
//
//    cIP.deltaPosition = deltaPosition;
//    pthread_t th;
//    pthread_create(&th, NULL,staticIncrDecrPositionHomingFunctionForThread,this);
//
//    return 0;
//}

// Set trapezoidal parameters
int TechnoSoftLowDriver::setSpeed(const double& _speed_mm_s){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    DPRINT("Chiamata setspeed");
    if(_speed_mm_s<=0 || _speed_mm_s>maxSpeed_mm_s){
        DERR("Speed = %f",_speed_mm_s);
        return -1;
    }
    speed_ms_s = _speed_mm_s;
    return 0;
}

int TechnoSoftLowDriver::setRatiOfNoise(const double& _ratiOfNoise){
    //printf("speed = %f, max speed = %f", _speed,maxSpeed);
    DPRINT("Chiamata setRatiOfNoise");
    if(_ratiOfNoise<0 || _ratiOfNoise>1){
        DERR("Ratio specified for noise = %f",_ratiOfNoise);
        return -1;
    }
    percNoise = _ratiOfNoise;
    return 0;
}

int TechnoSoftLowDriver::setMaxSpeed(const double& _maxspeed_mm_s){

    if(_maxspeed_mm_s<=0 || _maxspeed_mm_s<speed_ms_s){
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

int TechnoSoftLowDriver::moveAbsolutePosition(){

    // Stima grossolana tempo necessario per la movimentazione
//    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//    double tol = 30;
//    deltaT += (deltaT*tol/100);
//
//    double otalTimeInterval = 0;   // solo per far partire il ciclo while
//    struct timeval startTime,endTime;
//
//    gettimeofday(&startTime,NULL);

//    if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//    }

    DPRINT("moveAbsolutePosition: Thread partito");


    bool goahead = false;
    if(pthread_mutex_lock(&(mu))!=0){

    }
    if(position==(absolutePosition)){ //position: current position
//        if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//        }
        return 0;
    }
    
    if(absolutePosition<0){
        if(pthread_mutex_unlock(&(mu))!=0){

        }
        return -1;
    }
    // Quindi absolutePosition>=0 && absolutePosition <= LONG_MAX (perche' e' rappresentabile)
    // La slitta dunque si muovera' nella posizione [0,LONG_MAX]

    long initPosition = position; // mi prendo la posizione corrente del motorE
    actuatorIDInMotion = true;

    if(absolutePosition>initPosition)
        goahead = true;

    if(pthread_mutex_unlock(&(mu))!=0){

    }

    bool resetLimitSwicth = true;

    DPRINT("moveAbsolutePosition: appena prima del ciclo while");

    long tol = 150;

    while(std::labs(position-absolutePosition)>tol && !stopMotionCommand && !powerOffCommand){// L'incremento dovra' avvenire ad una determinata velocita'

        //DPRINT("moveAbsolutePosition: dentro il ciclo while: inizio prima del lock!!!");
        if(pthread_mutex_lock(&(mu))!=0){

        }

            if(goahead){
                position+=speed_ms_s;
                positionCounter+=speed_ms_s;
                positionEncoder+=speed_ms_s;
            }
            else{
                position-=speed_ms_s;
                positionCounter-=speed_ms_s;
                positionEncoder-=speed_ms_s;
            }

//            DPRINT("moveAbsolutePosition: position %ld",position);
//            DPRINT("moveAbsolutePosition: positionCounter %ld",positionCounter);
//            DPRINT("moveAbsolutePosition: positionEncoder %ld",positionEncoder);
//
//            DPRINT("moveAbsolutePosition: labs(position-absolutePosition) %ld",labs(position-absolutePosition));

            if(resetLimitSwicth){
                if (position >= 0){
                    LSNactive=false;
                }
                if (position<=LONG_MAX){
                    LSPactive=false;
                }
                resetLimitSwicth=false;
            }

            if(pthread_mutex_unlock(&(mu))!=0){

            }
        //DPRINT("moveAbsolutePosition: dentro il ciclo while: appena fuori il blocco del lock!!!");
        // Aggiornamento deltaT, stopMotion, stopPower
//        deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//        deltaT += (deltaT*tol/100);
//
//        gettimeofday(&endTime,NULL);
//        totalTimeInterval = ((double)endTime.tv_sec+(double)endTime.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
            usleep(1000); // Sleep for 5 milli second
    }
    //position=currentPosition;
    if(pthread_mutex_lock(&(mu))!=0){

    }

    actuatorIDInMotion = false;
    stopMotionCommand = false;
    motionscalled--;

    if(position<0){ // nel qual caso absolutePosition dato in input ==0
        LSNactive = true;
    }
    if(position>LONG_MAX){  // nel qual caso absolutePosition dato in input == LONG_MAX
        LSPactive = true;
    }
    
    if(pthread_mutex_unlock(&(mu))!=0){

    }
    return 0;
}

void* TechnoSoftLowDriver::staticMoveAbsolutePositionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->moveAbsolutePosition();

    pthread_exit(NULL); // Perche' e' la funzione chiamata direttamente dal thread
}

int TechnoSoftLowDriver::moveAbsoluteSteps(const long& absPosition){

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
    DPRINT("moving Absolute steps. Axis: %d, absPosition %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,absPosition,speed_ms_s,acceleration_mm_s2,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveAbsolute(absPosition, speed_mm_s, acceleration_mm_s2, movement, referenceBase)){
//        DERR("error absolute step moving");
//        return -2;
//    }
    if(pthread_mutex_lock(&(mu))!=0){

    }
    motionscalled++;
    if(pthread_mutex_unlock(&(mu))!=0){

    }

    if(motionscalled>1){
        stopMotion();
        usleep(100000); // Attendi che la corrente movimentazione si fermi
    }

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/20))
        return -1;

    pthread_t th;
    absolutePosition=absPosition;

    pthread_create(&th, NULL,staticMoveAbsolutePositionForThread,(void*)this);

    return 0;
}

int TechnoSoftLowDriver::getHighSpeedHoming(double& _highSpeedHoming_mm_s){

    DPRINT("Valore letto dell'high speed homing %f:", highSpeedHoming_mm_s);
    _highSpeedHoming_mm_s = highSpeedHoming_mm_s;
    return 0;
}

int TechnoSoftLowDriver::moveConstantVelocityHoming(){

    if(pthread_mutex_lock(&(mu))!=0){

    }

    // L'incremento deve avvenire ad una determinata velocita'
    if(position<=0 || LSNactive){
        DPRINT("La posizione in steps e' gia' <= 0. Non occorre spostarsi ancora indietro per effettuare di nuovo l'homing");
        if(pthread_mutex_unlock(&(mu))!=0){

        }
        return -1;
    }

    actuatorIDInMotion = true;

    if(pthread_mutex_unlock(&(mu))!=0){

    }
//    bool goahead = false;
//    if(highSpeedHoming_mm_s>=0)
//        goahead = true;

    while(!stopMotionCommand && !powerOffCommand && !LNStransition){

//            if(goahead){
//                position+=highSpeedHoming_mm_s;
//            }
//            else{
        if(pthread_mutex_lock(&(mu))!=0){

        }

        position+=highSpeedHoming_mm_s;
        positionCounter+=highSpeedHoming_mm_s;
        positionEncoder+=highSpeedHoming_mm_s;
        
        DPRINT("Movimentazione all'indietro: posizione corrente in steps: %ld",position);
//            }
        if(pthread_mutex_unlock(&(mu))!=0){

        }
        
//        if(resetLimitSwicth){
//            if (position > 0){
//                LSNactive=false;
//            }
//            if (position<LONG_MAX){
//                LSPactive=false;
//            }
//            resetLimitSwicth=false;
//        }

        // Aggiornamento deltaT, stopMotion, stopPower
//        deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//        deltaT += (deltaT*tol/100)
        usleep(1000); // Sleep for 1 milli second
    }
    if(pthread_mutex_lock(&(mu))!=0){

    }
        actuatorIDInMotion = false;
        stopMotionCommand = false;
    if(pthread_mutex_unlock(&(mu))!=0){

    }
    DPRINT("Posizione raggiunta dopo la rilevazione della transizione dello switch %ld",position);
    sleep(30);
    //LNStransition = false;
    return 0;
}

void* TechnoSoftLowDriver::staticMoveConstantVelocityHomingFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->moveConstantVelocityHoming();

    pthread_exit(NULL);
}

int TechnoSoftLowDriver::moveVelocityHoming(){
    
    //double highSpeedHoming_MicroSteps_s = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*highSpeedHoming_mm_s)/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    //double accelerationHoming_MicroSteps_s = round((N_ROUNDS_DEFAULT*STEPS_PER_ROUNDS_DEFAULT*CONST_MULT_TECHNOFT_DEFAULT*accelerationHoming_mm_s2/LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);
    DPRINT("(homing) moving velocity. Axis : %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,highSpeedHoming_mm_s,accelerationHoming_mm_s2,movementHoming,referenceBaseHoming);
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -1;
////    }
//    if(!TS_MoveVelocity(highSpeedHoming_mm_s, accelerationHoming_mm_s2, movementHoming, referenceBaseHoming)){
//        DERR("(homing) Error moving velocity ");
//        return -2;
//    }
    
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)){
        return -1;
    }

    pthread_t th;
    pthread_create(&th, NULL,staticMoveConstantVelocityHomingFunctionForThread,this);

    return 0;
}

int TechnoSoftLowDriver::moveAbsolutePositionHoming(){
    
    // Stima grossolana tempo necessario per la movimentazione
//    double deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//    double tol = 30;
//    deltaT += (deltaT*tol/100);
//
//    double totalTimeInterval = 0;   // solo per far partire il ciclo while
//    struct timeval startTime,endTime;
//
//    gettimeofday(&startTime,NULL);

//    if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//    }
    //bool goahead = false;
    if(pthread_mutex_lock(&(mu))!=0){

    }
    if(position==(absolutePosition)){ //position: current position
//        if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//        }
        DPRINT("Recupero homing non necessario");
        if(pthread_mutex_unlock(&(mu))!=0){

        }
        return 0;
    }
    
//    if(absolutePosition<0){
//        if(pthread_mutex_unlock(&(mu))!=0){
//
//        }
//        return -1;
//    }
    // Quindi absolutePosition>=0 && absolutePosition <= LONG_MAX (perche' e' rappresentabile)
    // La slitta dunque si muovera' nella posizione [0,LONG_MAX]

    long initPosition = position; // mi prendo la posizione corrente del motorE
    actuatorIDInMotion = true;

//    if(absolutePosition>initPosition){
//        goahead = true;
//    }

    if(pthread_mutex_unlock(&(mu))!=0){ //Sblocchiamo il mutex

    }
    
    DPRINT("Posizione corrente dal quale partire: %ld",position);
    DPRINT("Absolute position da raggiungere: %ld",absolutePosition);

    long tol = 150;
    bool resetLimitSwicth=true;
    while(labs(position-absolutePosition)>tol && !stopMotionCommand && !powerOffCommand){// L'incremento dovra' avvenire ad una determinata velocita'
        
        DPRINT("Posizione corrente durante il recupero: %ld",position);
        if(pthread_mutex_lock(&(mu))!=0){

        }

       
        DPRINT("Go ahead = true");
        position+=lowSpeedHoming_mm_s;
        positionCounter+=lowSpeedHoming_mm_s;
        positionEncoder+=lowSpeedHoming_mm_s;
        
//        else{
//            DPRINT("Go ahead = false");
//            position-=lowSpeedHoming_mm_s;
//            positionCounter-=lowSpeedHoming_mm_s;
//            positionEncoder-=lowSpeedHoming_mm_s;
//        }

        if(resetLimitSwicth){
            if (position >= 0){
                LSNactive=false;
            }
            if (position=<LONG_MAX){
                LSPactive=false;
            }
            resetLimitSwicth=false;
        }

        if(pthread_mutex_unlock(&(mu))!=0){

        }
        // Aggiornamento deltaT, stopMotion, stopPower
//        deltaT = fabs(((containerIncrementPosition*)arg)->deltaPosition/speed_ms_s); //[s]
//        deltaT += (deltaT*tol/100);
//
//        gettimeofday(&endTime,NULL);
//        totalTimeInterval = ((double)endTime.tv_sec+(double)endTime.tv_usec/1000000.0)-((double)startTime.tv_sec+(double)startTime.tv_usec/1000000.0);
        usleep(1000); // Sleep for 1 milli second
    }
    //position=currentPosition;
    if(pthread_mutex_lock(&(mu))!=0){

    }

    actuatorIDInMotion = false;
    stopMotionCommand = false;
    motionscalled--;

//    if(position==0){ // nel qual caso absolutePosition dato in input ==0
//        LSNactive = true;
//    }
//    if(position== LONG_MAX){  // nel qual caso absolutePosition dato in input == LONG_MAX
//        LSPactive = true;
//    }

    if(pthread_mutex_unlock(&(mu))!=0){

    }
    return 0;
}

void* TechnoSoftLowDriver::staticMoveAbsolutePositionHomingFunctionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->moveAbsolutePositionHoming();

    pthread_exit(NULL);
}

int TechnoSoftLowDriver::moveAbsoluteStepsHoming(const long& absPosition){

    DPRINT("(homing) moving absolute steps. Axis: %d, absPosition %d, speed=%f, acceleration %f, movement %d, referencebase %d",axisID,absPosition,speed_ms_s,acceleration_mm_s2,movement,referenceBase);
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_MoveAbsolute(absPosition, lowSpeedHoming_mm_s, accelerationHoming_mm_s2, movementHoming, referenceBaseHoming)){
//        DERR("(homing) Error absolute steps moving");
//        return -1;
//    }
    // Simulazione dialogo con il drive/motor

    if(pthread_mutex_lock(&(mu))!=0){

    }
    motionscalled++;
    if(pthread_mutex_unlock(&(mu))!=0){

    }

    if(motionscalled>1){
        stopMotion();
        usleep(100000); // Attendi che la corrente movimentazione si fermi
    }

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/20))
        return -1;

    pthread_t th;
    absolutePosition=absPosition;

    pthread_create(&th, NULL,staticMoveAbsolutePositionHomingFunctionForThread,this);

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
//    if(!TS_Stop()){
//        return -1;
//    }

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -1;

    if(pthread_mutex_lock(&(mu))!=0){

    }

    stopMotionCommand = true;

    if(pthread_mutex_unlock(&(mu))!=0){

    }

//    DPRINT("Motor with axis = %d is stopped, %s",axisID, TS_GetLastErrorText());
    DPRINT("Motor with axis = %d is stopped", axisID);
    return 0;
}

void* TechnoSoftLowDriver::staticStopMotionForThread(void* objPointer){ // Metodo statico chiamato eseguito direttamente dai threads

    //objPointer permettera' al thread di eseguire le funzione membro della classe TechnoSoftLowDriver
    ((TechnoSoftLowDriver*)objPointer)->stopMotion();

    pthread_exit(NULL);
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
//    if(!TS_Power(POWER_ON)){
//        //ERR("ALEDEBUG Error selecting axis");
//        return -2;
//    }

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -2;

    powerOffCommand = false;

//    /*	Wait for power stage to be enabled */
//    WORD sAxiOn_flag = 0;
//    while(sAxiOn_flag == 0){
//        /* Check the status of the power stage */
//        if(!TS_ReadStatus(REG_SRL, sAxiOn_flag)){
//
//            //ERR("ALEDEBUG Error TS_ReadStatus");
//            return -3;
//        }
//
//        sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
//    }
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
//    if(!TS_Power(POWER_OFF)){
//        return -1;
//    }

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -1;

    powerOffCommand = true;
    DPRINT("Motor with axis id = %d is power off",axisID);

    return 0;
}

int TechnoSoftLowDriver::deinit(){ // Identical to TechnoSoftLowDriver::stopPower()

    DPRINT("TechnoSoftLowDriver %d object is deallocated",axisID);
    return 0;
}

int TechnoSoftLowDriver::getCounter(double* deltaPosition_mm){

    //DPRINT("Reading COUNTER position");
    //long tposition;
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_GetLongVariable("TPOS", tposition)){
//        return -1;
//    }

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)){
        return -1;
    }

    //*deltaPosition_mm = (tposition*linear_movement_per_n_rounds)/(steps_per_rounds*const_mult_technsoft*n_rounds);
//    random_variable = (std::rand()/RAND_MAX)/1000; // Normalizzazione variabile random_variable

//    if(random_variable>0.0005){
//        pos=positionCounter+random_variable;
//    }
//    else{
//        pos=positionCounter-random_variable;
//    }

//    // This is the underlying integer random number generator
//    boost::mt19937 igen;
//    long pos = positionCounter;
//    // The second template parameter is the actual floating point
//    // distribution that the user wants
//    double stdv = abs(pos+pos*0.1);
//    boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
//        gen(igen, boost::normal_distribution<>(pos,stdv));
//
//    DPRINT("Real position counter %f, position counter with noise %ld",pos,gen());

    //double pos = positionCounter;
    //DPRINT("pos = %ld",pos);
//    long min = pos-(long)deltaNoise;
//    long max = pos+(long)deltaNoise;
    //long deltaNoise = (long)(pos*percNoise);
    //DPRINT("Delta noise = %ld",deltaNoise);

//    const long rangeMin = pos-deltaNoise;
//    const long rangeMax = pos+deltaNoise;
//
//    NumberDistribution distribution(rangeMin, rangeMax);
//    Generator numberGenerator(generator, distribution);

    //std::srand(std::time(0));
    //DPRINT("Real position encoder %ld, position encoder with noise %ld",pos,(long)(min+(max-min)*std::rand()/RAND_MAX));

//    *deltaPosition_mm = (tposition*linear_movement_per_n_rounds)/(steps_per_rounds*const_mult_technsoft*n_rounds);
//    double pos_mm = (pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
//    double deltaNoise = (pos_mm*percNoise);
//    const double rangeMin = pos_mm-deltaNoise;
//    const double rangeMax = pos_mm+deltaNoise;
//    NumberDistribution distribution(rangeMin, rangeMax);
//    Generator numberGenerator(generator, distribution);
//    *deltaPosition_mm = numberGenerator();
    double pos = positionCounter;
    *deltaPosition_mm = (pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
    return 0;
}

//double getRandomDoubleUsingNormalDistribution(double mean, double sigma)
//{
// //typedef normal_distribution<> NormalDistribution;
// typedef boost::mt19937 RandomGenerator;
// typedef boost::variate_generator GaussianGenerator;
//
//  /** Initiate Random Number generator with current time */
//  static RandomGenerator rng(static_cast (time(0)));
//
//  /* Choose Normal Distribution */
//  typedef normal_distribution<> gaussian_dist(mean, sigma);
//
//  /* Create a Gaussian Random Number generator
//   *  by binding with previously defined
//   *  normal distribution object
//   */
//  GaussianGenerator generator(rng, gaussian_dist);
//
//  // sample from the distribution
//  return generator();
//}


int TechnoSoftLowDriver::getEncoder(double* deltaPosition_mm){

    //DPRINT("Reading ENCODER position");
//    long aposition;
//    if(!TS_GetLongVariable("APOS", aposition)){
//        return -1;
//    }
    //DPRINT("E' questo il getEncoder???????????????");
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)){
        return -1;
    }

//    random_variable = (std::rand()/RAND_MAX)/1000; // Normalizzazione variabile random_variable

//    std::srand(std::time(0));
//
//    long pos = positionEncoder;
//    long min = pos-(long)deltaNoise;
//    long max = pos+(long)deltaNoise;

    //Rumore gaussiano: e' necessario lo standard C++ 2011
//    double stdv = abs(pos+pos*0.001);
//    std::normal_distribution<double> distribution(pos,stdv);
//    pos = (long)(distribution(generator));

//    if(random_variable>0.0005){
//        pos=positionEncoder+random_variable;
//    }
//    else{
//        pos=positionEncoder-random_variable;
//    }

    // This is the underlying integer random number generator
//    boost::mt19937 igen;
//    long pos= positionEncoder;
//    // The second template parameter is the actual floating point
//    // distribution that the user wants
//    double stdv = abs(pos+pos*0.1);
////    boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
////        generator(boost::mt19937(time(0)), boost::normal_distribution<>(pos,stdv));
//
//    DPRINT("Real position encoder %ld, position encoder with noise %f",pos,getRandomDoubleUsingNormalDistribution(pos,stdv));
////    *deltaPosition_mm = (aposition*linear_movement_per_n_rounds)/(n_encoder_lines*n_rounds);

    //std::srand(std::time(0));

    double pos = positionEncoder;
//    long min = pos-(long)deltaNoise;
//    long max = pos+(long)deltaNoise;
//    long deltaNoise = (long)(pos*percNoise);
//
//    const long rangeMin = pos-deltaNoise;
//    const long rangeMax = pos+deltaNoise;
//
//    NumberDistribution distribution(rangeMin, rangeMax);
//    Generator numberGenerator(generator, distribution);

//    for(int i=0;i<10;i++){
//    std::cout << numberGenerator() << std::endl;
//    }

    //DPRINT("Real position encoder %ld, position encoder with noise %ld",pos,(long)(min+(max-min)*std::rand()/RAND_MAX));

    //*deltaPosition_mm = (numberGenerator()*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
    
    if(percNoise>0){
        double pos_mm = (pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
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
        *deltaPosition_mm=(pos*linear_movement_per_n_rounds)/(steps_per_rounds*n_rounds*const_mult_technsoft);
    
    //DPRINT("Real position encoder %f millimeter", *deltaPosition_mm);
   
    return 0;
}

int TechnoSoftLowDriver::getLVariable(std::string& nameVar, long& var) {
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_GetLongVariable(nameVar.c_str(), var)){
//        return -1;
//    }

    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -1;

    var = position;

    return 0;
}

int TechnoSoftLowDriver::resetCounter(){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_Execute("SAP 0")){
//        return -1;
//    }
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)){
        return -1;
    }
    if(pthread_mutex_lock(&(mu))!=0){

    }
        positionCounter = 0;
        //position = 0;
    if(pthread_mutex_unlock(&(mu))!=0){

    }
    return 0;
}

void* TechnoSoftLowDriver::staticResetCounterForThread(void* objPointer){

    ((TechnoSoftLowDriver*)objPointer)->resetCounter();
    pthread_exit(NULL);
}


int TechnoSoftLowDriver::resetEncoder(){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_Execute("APOS=0")){
//        return -1;
//    }
    // Simulazione dialogo con il drive/motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)){
        return -1;
    }

    if(pthread_mutex_lock(&(mu))!=0){

    }
    positionEncoder = 0;
    if(pthread_mutex_unlock(&(mu))!=0){

    }

    return 0;
}

void* TechnoSoftLowDriver::staticResetEncoderForThread(void* objPointer){

    ((TechnoSoftLowDriver*)objPointer)->resetEncoder();
    pthread_exit(NULL);
}

int TechnoSoftLowDriver::setEventOnLimitSwitch(short lswType, short transitionType, BOOL waitEvent, BOOL enableStop){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_SetEventOnLimitSwitch(lswType, transitionType, waitEvent, enableStop)){
//	return -1;
//    }
    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)) {
        return -1;
    }

    return 0;
}

int TechnoSoftLowDriver::setEventOnMotionComplete(BOOL waitEvent, BOOL enableStop){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_SetEventOnMotionComplete(waitEvent,enableStop)){
//	return -2;
//    }
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)) {
        return -1;
    }

    return 0;
}

//int TechnoSoftLowDriver::checkEvent(BOOL& event){
////    if(!TS_SelectAxis(axisID)){
////        DERR("failed to select axis %d",axisID);
////        return -1;
////    }
////    if(!TS_CheckEvent(event)){
////        return -1;
////    }
//    int random_variable = std::rand();
//    if(random_variable<1*(RAND_MAX/100)) {
//        return -1;
//    }
//
//    if(position <= -epsylon){
//        LNStransition = true; // Evento che segnale il limit switch transition
//        // Occorre "bloccare" il motion, ovvero terminare il thread che si sta occupando del motion
//        //stopMotionCommand = true; // Il thread terminera' cosicche' il motion stesso terminera'...
//        event = true;
//    }
//    else{
//        LNStransition = false;
//        event = false;
//    }
//    return 0;
//}

int TechnoSoftLowDriver::checkEvent(BOOL& event){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_CheckEvent(event)){
//        return -1;
//    }

    //Evento che segnale il limit switch transition

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)) {
        return -1;
    }

//    if(pthread_mutex_lock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//    }
    if(pthread_mutex_lock(&(mu))!=0){

        }
    if(position <= -epsylon){
        DPRINT("Rilevata posizione <= 0: %ld", position);
        LNStransition = true;// Occorre "bloccare" il motion, ovvero terminare il thread che si sta occupando del motion
        cap_position = position;
        event = true;        // comunichiamo alla funzione homing l'avvenuta transizione del LSN
        //sleep(30);              
        //stopMotionCommand=true; //Serve a far terminare la movimentazione  
    }
    else{
        DPRINT("Posizione riscontrata durante il checking: %ld", position);
        LNStransition = false;
        event = false;
        //sleep(30);
    }
    if(pthread_mutex_unlock(&(mu))!=0){

    }
    
//    if(pthread_mutex_unlock(&(((containerIncrementPosition*)arg)->mu))!=0){
//
//    }

    return 0;
}

int TechnoSoftLowDriver::getStatusOrErrorReg(const short& regIndex, WORD& contentRegister, std::string& descrErr){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!TS_ReadStatus(regIndex,contentRegister)){
//
//        //DERR("Error at the register reading: %s",TS_GetLastErrorText());
//        //descrErr.assign(TS_GetLastErrorText());
//        descrErr=descrErr+" Error reading status: "+TS_GetLastErrorText();
//        return -2;
//    }
    // Simulazione dialogo con il drive motor

    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100)) {
        //descrErr=descrErr+" Error reading status: "+TS_GetLastErrorText();
        return -1;
    }

    // Inizializzazione "casuale" codici allarmi:
//    if(alarmsInfoRequest && regMERrequest){
//        for(uint16_t i=0; i<sizeof(contentRegister)*8; i++){
//            random_variable = std::rand();
//            if(random_variable<p*(RAND_MAX/100))
//                contentRegister |= ((WORD)1<<i);
//        }
//        //contentRegister = contentRegMER;
//        return 0;
//    }
    if(alarmsInfoRequest && regMERrequest){
        contentRegister=0;
        return 0;
    }
//    else if(alarmsInfoRequest && regSRHrequest){
//        for(uint16_t i=10; i<12; i++){
//            random_variable = std::rand();
//            if(random_variable<p*((RAND_MAX)/100))
//                contentRegSRH |= ((WORD)1<<i);
//        }
//        contentRegister = contentRegSRH;
//        return 0;
//    }
    else if(alarmsInfoRequest && regSRHrequest){
        contentRegister=0;
        return 0;
    }
    // Inizializzazione "casuale" codici stati:
    else if(stateInfoRequest && regSRHrequest){ // Non posso modificare il contenuto dei bit 10, 11 di SRH
        for(uint16_t i=0; i<sizeof(contentRegSRH)*8; i++){
            if(i==5 || i==6 || i==7 ||i==12 || i==14 || i==15){
                random_variable = std::rand();
                if(random_variable<p*(RAND_MAX/100))
                    contentRegSRH |= ((WORD)1<<i);
            }
        }
        contentRegister = contentRegSRH;
        return 0;
    }
    else if(stateInfoRequest && regSRLrequest){
        random_variable = std::rand();
        if(random_variable<p*(RAND_MAX/100)){
            contentRegSRL |= ((WORD)1<<10);
        }
        random_variable = std::rand();
        if(random_variable<p*(RAND_MAX/100)){
            contentRegSRL |= ((WORD)1<<15);
        }
        contentRegister = contentRegSRL;
        return 0;
    }
    return -2;
}


int TechnoSoftLowDriver::resetFault(){

//    if(!TS_ResetFault()){
//         return -2;
//         // Note: the drive-motor will return to FAULT status (SRH.15=1) if there are
//         // errors when the function is executed)
//    }
    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -1;

    //************************************************** DA GESTIRE ************************************************************
    //    if(!TS_ResetFault()){
//         return -2;
//         // Note: the drive-motor will return to FAULT status (SRH.15=1) if there are
//         // errors when the function is executed)
//    }

    contentRegMER = 0;

    return 0;
}

int TechnoSoftLowDriver::selectAxis(){

//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -1;

    return 0;
}

// retrieve firmware version of the active drive
int TechnoSoftLowDriver::getFirmwareVers(char* firmwareVers){
//    if(!TS_SelectAxis(axisID)){
//        DERR("failed to select axis %d",axisID);
//        return -1;
//    }
//    if(!MSK_GetDriveVersion(firmwareVers)){
//        DERR("Errore lettura versione driver");
//        return -2;
//    }
    // Simulazione dialogo con il drive motor
    int random_variable = std::rand();
    if(random_variable<p*(RAND_MAX/100))
        return -1;
    char msg[] = "firmware version is ...";
    firmwareVers = msg;

    return 0;
}
