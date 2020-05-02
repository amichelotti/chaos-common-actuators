#include "iostream"
#include "stdio.h"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "../ActuatorTechnoSoft.h"
#include <pthread.h>
#include <fstream>
#include <sstream>
#include <sstream>
#include <boost/regex.hpp>

using namespace common::actuators::models;
#define USAGE \
  printf("**************Usage is:%s <dev/tty> <technosoft configuration> <axis> <move position in mm>*************\n",argv[0]);

struct homingData{
    int a;
    int b;
    common::actuators::AbstractActuator *obj;
    int c;
};
struct checkData{
    int axisID;
    double duration;
    common::actuators::AbstractActuator *obj;
};

int setParameter(common::actuators::AbstractActuator *OBJ,int axisID,std::string parName,std::string value){
    
    if(OBJ->setParameter(axisID,parName,value)<0){
        DERR("************** Error returned by setParameter %s **************",parName.c_str());
        return -1;
    }
    //DPRINT("************** Value of %s updated to %s **************",parName.c_str(),value.c_str());

    return 0;  
}

int getParameter(common::actuators::AbstractActuator *OBJ,int axisID,std::string parName,std::string& resultString ){
    
    if(OBJ->getParameter(axisID,parName,resultString)<0){
        DERR("************** Error returned by getParameter %s **************",parName.c_str());
        return -1;
    }
    return 0;  
}

void* checkProcedures(void* p){
    
    checkData* pstruct=(checkData*) p;
    int axisID = pstruct->axisID;
    double duration = pstruct->duration;
    common::actuators::AbstractActuator *OBJ=pstruct->obj;
    
    struct timeval startTimeForMotor1,endTimeForMotor1;

    double total_time_interval=0;
    double position_mm_encoder;
    double position_mm_counter;
    double position_mm_potentiometer;
    int resp;
    std::string desc1;
    std::string desc2;
    uint64_t alarms;
    int state;
    
    gettimeofday(&startTimeForMotor1,NULL);
      
        while(total_time_interval<=duration){
            
            if((resp=OBJ->getState(axisID,&state, desc1))<0){
                DERR("************** Error returned by getState operation, code error %d **************",resp);
                sleep(10);
                //* errPtr = -5;
            }
            
            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &position_mm_counter))<0){
                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
                sleep(10);
                //* errPtr = -5;
            }
            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &position_mm_encoder))<0){
                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
                sleep(10);
                //* errPtr = -5;
            }
            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_POTENTIOMETER, &position_mm_potentiometer))<0){
                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
                sleep(10);
                //* errPtr = -5;
            }
            
            if((resp=OBJ->getAlarms(axisID,&alarms,desc2))<0){
                DERR("************** Error reading alarms ***************");
            }
//            if((resp=OBJ->getState(axisID,&state,desc1))<0){
//                DERR("************** Error reading alarms ***************");
//            }
            DPRINT("************** State of axisID partita: %s **************",desc1.c_str());
            DPRINT("************** Alarms of axisID : %s  **************",desc2.c_str());
            DPRINT("************** Position encoder of axisID : %4.13f **************",position_mm_encoder);
            DPRINT("************** Position counter of axisID : %4.13f **************",position_mm_counter);
            DPRINT("************** Position potentiometer of axisID : %4.13f **************",position_mm_potentiometer);
//            DPRINT("************** Position counter of axisID 14: %4.13f  **************",position_mm_counter);
            //DPRINT("************** State of axisID 14: %s  **************",desc1.c_str());
            
            //DPRINT("************** Code Alarms of axisID 14: %u **************",alarms);
            
            gettimeofday(&endTimeForMotor1,NULL);
            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);

            DPRINT("total_time_interval: %f",total_time_interval);
            
            usleep(1000000); // lettura ogni decimo di secondo...
        }
    return NULL;
}

void* homingProcedures(void *p){ 
    
    homingData* pstruct=(homingData*) p;
    int respHoming = pstruct->a;
    int numHoming = pstruct->b;
    common::actuators::AbstractActuator *OBJ = pstruct->obj;
    int axisID=pstruct->c;
   
    struct timeval startTimeForMotor1,endTimeForMotor1;
    double total_time_interval=0;
    
    double durationChecking = 120; // secondi
//        
    checkData hd2;
    hd2.axisID=axisID;
    hd2.duration=durationChecking;
    hd2.obj=OBJ;
    //checkProcedures((void*)&hd2);
    
    gettimeofday(&startTimeForMotor1,NULL);
    
    for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
            // con successo o insuccesso
        DPRINT("************* Procedura di homing n. %d iniziata *************",i);
        while(respHoming){ // Finche' la procedura di homing non e' completata con successo
            respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::defaultHoming); // Il parametro in ingresso alla funzione non e' piu letto
            usleep(1000); // FREQUENZA DI 100 ms
            if(respHoming<0){
                DERR("***************Procedura di homing terminata con errore n. %d ***************",respHoming);
                break;
            }
            gettimeofday(&endTimeForMotor1,NULL);
            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
            checkProcedures((void*)&hd2);
            //DPRINT("************Valore ritornato dalla funzione di homing %d ***************",respHoming);
            //DPRINT("************Numero chiamata procedura di homing %d ***************",i);
        }
        if(respHoming==0){
            DPRINT("************ Procedura di homing n. %d terminata con successo ***************",i);
            sleep(60);
        }
        respHoming = 1;
        //usleep(5000000);
    }
    return NULL;
}


struct stopMotionStruct{
    int axisID;
    common::actuators::AbstractActuator *obj;
};

struct resetAlarmsStruct{
    int axisID;
    common::actuators::AbstractActuator *obj;
};

void* resetAlarmsProcedure(void* p){
    
    resetAlarmsStruct* ptr = (resetAlarmsStruct*)p;
    int axisID=ptr->axisID;
    common::actuators::AbstractActuator *OBJ=ptr->obj;
    
    struct timeval startTimeForMotor1,endTimeForMotor1;
    double total_time_interval=0;
    
    gettimeofday(&startTimeForMotor1,NULL);
    int resp;
    if((resp=OBJ->resetAlarms(axisID,0))<0){
            DERR("************** Error returned by movement operation, code error %d **************",resp);
            sleep(10);
            //* errPtr = -5;
    }
    DPRINT("************** Comando di reset alarms eseguito **************");
    
    gettimeofday(&endTimeForMotor1,NULL);
    total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
    
    // ********* Salvataggio su file dell'intervallo di tempo trascorso *********
    std::fstream myfile;
    std::stringstream stream;
    std::string filename = "//home/caschera/chaos_bundle/chaosframework/chaos-distrib-i686-linux26/fileProva.txt";
    stream<<filename.c_str();
    std::string newfilename = stream.str();
    
    myfile.open(newfilename.c_str(),std::ios::out);
    if(myfile.is_open()){
        // Descrizione testo effettuato
        myfile<<"Intervallo di tempo impiegato dalla procedura di homing a fermarsi: "<<std::endl<<std::endl;
        myfile<<total_time_interval<<std::endl;
        myfile.close();
    }
    else{
        std::cout<<"Errore nell'apertura del file"<<std::endl;
    }
    return NULL;
}

void* stopMotionProcedure(void* p){
    
    stopMotionStruct* ptr = (stopMotionStruct*)p;
    int axisID=ptr->axisID;
    common::actuators::AbstractActuator *OBJ=ptr->obj;
    
    int resp;
    
    if((resp=OBJ->stopMotion(axisID))<0){
            DERR("************** Error returned by movement operation, code error %d **************",resp);
            sleep(10);
            //* errPtr = -5;
    }
    DPRINT("************** Comando di stop eseguito **************");
    return NULL;
}

int procedura(common::actuators::AbstractActuator *OBJ,int numSeq){
    
    DPRINT("************** Procedura n. %d starting **************",numSeq);
    int axisID = 14;

    // MOVIMENTAZIONE ASSE 14
    if(OBJ->poweron(axisID,1)<0){
        DERR("**************Error returned by poweron operation **************");
        sleep(10);
        //* errPtr = -5;
    }
          
    double durationChecking=30;

    checkData hd2;
    hd2.axisID=axisID;
    hd2.duration=durationChecking;
    hd2.obj=OBJ;
    checkProcedures((void*)&hd2);
    
    int resp;
    if((resp=OBJ->moveRelative(axisID,5))<0){
        DPRINT("************** Error returned by movement operation, code error %d **************",resp);
    }
    
    sleep(20);
    std::string nameParameter = " probErrorOperation    ";
    setParameter(OBJ,axisID,nameParameter,"0.8");
    
    std::string valueStr;
    if(getParameter(OBJ,axisID,nameParameter,valueStr)>=0)
        DPRINT("************** Value of  %s = %s **************",nameParameter.c_str(),valueStr.c_str());

//    nameParameter = " alarmsInterval    ";
//    setParameter(OBJ,axisID,nameParameter,"120");
//    if(getParameter(OBJ,axisID,nameParameter,valueStr)>=0)
//        DPRINT("************** Value of  %s = %s **************",nameParameter.c_str(),valueStr.c_str());
    
    sleep(10);
    
    hd2.duration=600;
    checkProcedures((void*)&hd2);
    
//    DPRINT("************** Prima movimentazione relativa in AVANTI **************");
//    sleep(5);

    
//    durationChecking=30;
//    checkProcedures((void*)&hd2);
//    
//    DPRINT("************** Operazione di hard reset eseguita fra 5 secondi **************");
//    sleep(5);
//    int resp;
//    if((resp=OBJ->hardreset(axisID, false))<0){
//        DPRINT("************** Error returned by hardreset operation**************");
//    }
//    
//    durationChecking=30;
//    checkProcedures((void*)&hd2);
    
    return 0;
}

void* function1(void* str){

    int ret;
    char* strInit =(char*)str;
   
    common::actuators::AbstractActuator *OBJ = new common::actuators::models::simul::ActuatorTechnoSoft();

    // INIZIALIZZAZIONE CANALE
    if((ret=OBJ->init((void*)strInit))!=0){
        DERR("*************Cannot init channel. In fact the value returned is %d****************",ret);

    }
    //PROCEDURA DI CONFIGURAZIONE MOTORI, SEMPRE SULLO STESSO OGGETTO !!!

    std::string strConfig14 = "14,/home/caschera/chaos_bundle/common/actuators/models/technosoft/conf/1setup001.t.zip";
    if((ret=OBJ->configAxis((void*)strConfig14.c_str()))!=0){
        DERR("*************Cannot configure axis. In fact the value returned is %d****************",ret);
        sleep(120);
    }
    else{ // Invio comandi ai motori (axisID = 14)
        
        int numVolteProcedura=1;
        for(int i=1;i<=numVolteProcedura;i++){
            procedura(OBJ,i);
        }

        return 0;
    }
    return NULL;
}



bool validate_card_format(const std::string& s){
    static const boost::regex e("(\\d{4}[-]){3}\\d{4}"); // Formato che la stringa in input deve rispettare: 1234-5678-0321-8756
    return regex_match(s, e);
}

// match any format with the regular expression:
const boost::regex e("\\A(\\d{3,4})[- ]?(\\d{4})[- ]?(\\d{4})[- ]?(\\d{4})\\z");
const boost::regex str("(\\w+)\\W+(\\w+)");

static const boost::regex driver_match1("(\\d+),(\\d+),(\\d+),(\\d),(\\d+),\\d*.\\d*,(.+)");
static const boost::regex driver_match2("(\\d+),(.+)");

//"14,/home/caschera/chaos_bundle/common/actuators/models/technosoft/conf/1setup001.t.zip"

// \A Matches the beginning of the string
// {min,max} The preceding item is matched at least min times, but not more than max times
// \z Matches the end of the string
// ? The question mark indicates zero or one occurrences of the preceding element. For example, colou?r matches both "color" and "colour".

const std::string machine_format("\\1\\2\\3\\4");
const std::string human_format("\\1-\\2-\\3-\\4");

std::string machine_readable_card_number(const std::string s){
    return regex_replace(s, e, machine_format, boost::match_default | boost::format_sed);
}
std::string human_readable_card_number(const std::string s){
    return regex_replace(s, e, human_format, boost::match_default | boost::format_sed);
}

int main(int argc,const char* argv[]){

 

    int hostID;
    int btType;
    int baudrate;

    char sinit1[256];
    //char sinit2[256];
    if(argc!=5){
        USAGE;
        return -1;
    }
    // Inizializzazione parametri ASSE 1
//    dev1=argv[1];        // [string], <dev/tty>
//    conf1=argv[2];       // [string], <technosoft configuration>
//    axis1=atoi(argv[3]); // [int], <axis>
//    pos1=atof(argv[4]);  // [float], <move position in mm>
    hostID =   atoi(argv[1]);
    btType =   atoi(argv[2]);
    baudrate = atoi(argv[3]);
    const char* dev1 = argv[4];        // [string], <dev/tty>

    //PRINT("************ using axis %d, moving of %f mm**************",axis1,pos1);
    sprintf(sinit1,"%d,%d,%d,%s",hostID,btType,baudrate,dev1);

    pthread_t th1;
    DPRINT("%s",sinit1);
    DPRINT("main eseguito");
    pthread_create(&th1,NULL,function1,&sinit1[0]);
    pthread_join(th1,NULL); //pthread_join modifica il contenuto del puntatore resp_th_1
    
//    std::string cardNumber;
//    cardNumber.assign("1234-5678-9876-0987");
//    bool resp=validate_card_format(cardNumber);
//    if(resp){
//        DPRINT("************ Formato carta corretto **************");
//    }
//    else{
//        DPRINT("************ Formato carta non corretto **************");
//    }
//    
//    DPRINT("Machine readable card number: %s",machine_readable_card_number(cardNumber).c_str());
//    DPRINT("Human readable card number: %s",human_readable_card_number(cardNumber).c_str());

    return 0;
}
