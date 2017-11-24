#include "iostream"
#include "stdio.h"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "../ActuatorTechnoSoft.h"
#include <pthread.h>
#include <fstream>
#include <sstream>



#define AXISID_TEST 14

using namespace common::actuators::models;
#define USAGE \
  printf("**************Usage is:%s <dev/tty> <technosoft configuration> <axis> <move position in mm>*************\n",argv[0]);

// HOMING OPERATION
//        int respHoming=1; // Operazione di homing non conclusa
//        int numHoming = 1;
        
        
        
//        DPRINT("************** Homing operation starting for axis AXISID_TEST. Total homing procedure = %d **************", numHoming);
//        sleep(10);
        
//void homingProcedures(int respHoming,int numHoming,common::actuators::AbstractActuator *OBJ,int axisID){ 
//    for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
//            // con successo o insuccesso
//        DPRINT("************* Procedura di homing n. %d iniziata *************",i);
//        while(respHoming){ // Finche' la procedura di homing non e' completata con successo
//            respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::defaultHoming); // Il parametro in ingresso alla funzione non e' piu letto
//            usleep(1000); // FREQUENZA DI 100 ms
//            if(respHoming<0){
//                DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
//                break;
//            }
//        }
//        if(respHoming==0){
//            DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
//        }
//        respHoming = 1;
//        usleep(5000000);
//    }
//}
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
}

void* homingProcedures(void *p){ 
    
    DPRINT("Starting homing procedure");
    
    homingData* pstruct=(homingData*) p;
    int respHoming = pstruct->a;
    int numHoming = pstruct->b;
    common::actuators::AbstractActuator *OBJ = pstruct->obj;
    int axisID=pstruct->c;
    
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
            //gettimeofday(&endTimeForMotor1,NULL);
            //total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
            //checkProcedures((void*)&hd2);
            //DPRINT("************Valore ritornato dalla funzione di homing %d ***************",respHoming);
            //DPRINT("************Numero chiamata procedura di homing %d ***************",i);
        }
        if(respHoming==0){
            DPRINT("************ Procedura di homing n. %d terminata con successo ***************",i);
            sleep(20);
        }
        respHoming = 1;
        //usleep(5000000);
    }
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
}

int getParameter(common::actuators::AbstractActuator *OBJ,int axisID,std::string parName,std::string& resultString ){
    
    if(OBJ->getParameter(axisID,parName,resultString)<0){
        DERR("************** Error returned by getParameter %s **************",parName.c_str());
        return -1;
    }
    DPRINT("************** Value of %s = %s **************",parName.c_str(),resultString.c_str());

    return 0;  
}

int procedura(common::actuators::AbstractActuator *OBJ,int numSeq){
    
    DPRINT("************** Procedura n. %d starting **************",numSeq);
    int axisID = AXISID_TEST;

    // MOVIMENTAZIONE ASSE AXISID_TEST
    if(OBJ->poweron(axisID,1)<0){
        DERR("**************Error returned by poweron operation **************");
//        sleep(10);
        //* errPtr = -5;
        
    }
   
//    sleep(5);
        
    DPRINT("************** Prima movimentazione relativa in AVANTI **************");
    sleep(5);
    int resp;
    if((resp=OBJ->moveRelativeMillimeters(axisID,50))<0){
        DPRINT("************** Error returned by movement operation, code error %d **************",resp);
    }
//    sleep(120);
        
    double durationChecking=25;
    checkData hd2;
    hd2.axisID=axisID;
    hd2.duration=durationChecking;
    hd2.obj=OBJ;
    checkProcedures((void*)&hd2);
    
//    if((resp=OBJ->hardreset(axisID, false))<0){
//        DPRINT("************** Error returned by hardreset operation**************");
//    }
    
    DPRINT("************** Comando power off fra 5 s **************");
    sleep(5);
    
    if(OBJ->poweron(axisID,0)<0){
        DERR("**************Error returned by poweron operation **************");
//        sleep(10);
        //* errPtr = -5;
        
    }
   
    durationChecking = 10;
    checkProcedures((void*)&hd2);
    
    
    // controllo correttezza funzioni getParameters
  
    DPRINT("************** Comando power on fra 5 s **************");
    sleep(5);
    if(OBJ->poweron(axisID,1)<0){
        DERR("**************Error returned by poweron operation **************");
        sleep(10);
    }
    
    durationChecking = 120;
    checkProcedures((void*)&hd2);
   
    return 0;
}

void* function1(void* str){

    int ret;
    char* strInit =(char*)str;

    common::actuators::AbstractActuator *OBJ = new ActuatorTechnoSoft();

    // INIZIALIZZAZIONE CANALE
    if((ret=OBJ->init((void*)strInit))!=0){
        DPRINT("*************Cannot init channel with (%s) In fact the value returned is %d****************",strInit,ret);

    }
    //PROCEDURA DI CONFIGURAZIONE MOTORI, SEMPRE SULLO STESSO OGGETTO !!!

    std::string strConfig = "14,/home/caschera/chaos_bundle/common/actuators/models/technosoft/conf/1setup001.t.zip";
    if((ret=OBJ->configAxis((void*)strConfig.c_str()))!=0){
        DERR("*************Cannot configure axis. In fact the value returned is %d****************",ret);
        sleep(10);
    }

    else{ // Invio comandi ai motori (axisID = AXISID_TEST)
        
        int numVolteProcedura=1;
        for(int i=1;i<=numVolteProcedura;i++){
            procedura(OBJ,i);
        }
       
        return 0;
    }
}

int main(int argc,const char* argv[]){

    int axis1ID;
    int axis2;

    double pos1,pos2;
    double rpos=-1000,rpos1=-1000;
    int ret;
    int status;
    int hostID;
    int btType;
    int baudrate;

    const char *dev1,*conf1;
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
    dev1 = argv[4];        // [string], <dev/tty>

    //PRINT("************ using axis %d, moving of %f mm**************",axis1,pos1);
    sprintf(sinit1,"%d,%d,%d,%s",hostID,btType,baudrate,dev1);
    
//    std::string s = "Boost Libraries";
//    boost::regex expr("\\w+\\s\\w+");
//    std::cout << std::boolalpha << boost::regex_match(s, expr) << '\n';
//    
    
    //common::actuators::AbstractActuator*mySlit1 = NULL;

//    // Inizializzazione parametri ASSE 2
//    dev2="/dev/ttyr1d";        // [string], <dev/tty>
//    conf2=conf1;       // [string], <technosoft configuration>
//    axis2=15; // [int], <axis>
//    pos2=pos1;  // [float], <move position in mm>
//    //PRINT("************ using axis %d, moving of %f mm**************",axis2,pos2);
//    sprintf(sinit2,"%s,myslit2,%s,%d",dev2,conf2,axis2);
//    common::actuators::AbstractActuator*mySlit2 = NULL;

    pthread_t th1;
    pthread_t th2;
    DPRINT("%s",sinit1);
    DPRINT("main eseguito");
    //DPRINT("%s",sinit1);
    //function1(&sinit1[0]);

    pthread_create(&th1,NULL,function1,&sinit1[0]);
//
//    dev2=dev1;
//    conf2=conf1;
//    axis2=15;
//    pos2 = pos1;
//    sprintf(sinit2,"%s,myslit2,%s,%d,%d",dev2,conf2,axis2,hostID);
//    DPRINT("%s",sinit2);
//    pthread_create(&th2,NULL,function2,&sinit2[0]);

    //void* resp_th_1;
    //void* resp_th_2;

    pthread_join(th1,NULL); //pthread_join modifica il contenuto del puntatore resp_th_1

    return 0;
}