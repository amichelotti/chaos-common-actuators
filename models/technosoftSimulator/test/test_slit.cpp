#include "iostream"
#include "stdio.h"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "../ActuatorTechnoSoft.h"
#include <pthread.h>
#include <fstream>
#include <sstream>

using namespace common::actuators::models::simul;
#define USAGE \
  printf("**************Usage is:%s <dev/tty> <technosoft configuration> <axis> <move position in mm>*************\n",argv[0]);

// HOMING OPERATION
//        int respHoming=1; // Operazione di homing non conclusa
//        int numHoming = 1;
        
        
        
//        DPRINT("************** Homing operation starting for axis 14. Total homing procedure = %d **************", numHoming);
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

void* homingProcedures(void *p){ 
    
    homingData* pstruct=(homingData*) p;
    int respHoming = pstruct->a;
    int numHoming = pstruct->b;
    common::actuators::AbstractActuator *OBJ = pstruct->obj;
    int axisID=pstruct->c;
   
    struct timeval startTimeForMotor1,endTimeForMotor1;
    double total_time_interval=0;
    
    gettimeofday(&startTimeForMotor1,NULL);
    
    for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
            // con successo o insuccesso
        DPRINT("************* Procedura di homing n. %d iniziata *************",i);
        while(respHoming){ // Finche' la procedura di homing non e' completata con successo
            respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::homing2); // Il parametro in ingresso alla funzione non e' piu letto
            usleep(1000); // FREQUENZA DI 100 ms
            if(respHoming<0){
                DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
                break;
            }
            gettimeofday(&endTimeForMotor1,NULL);
            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
            
            //DPRINT("************Valore ritornato dalla funzione di homing %d ***************",respHoming);
            //DPRINT("************Numero chiamata procedura di homing %d ***************",i);
        }
        if(respHoming==0){
            DPRINT("************ Procedura di homing n. %d terminata con successo ***************",i);
        }
        respHoming = 1;
        //usleep(5000000);
    }
//    std::fstream myfile;
//    std::stringstream stream;
//    std::string filename = "//home/caschera/chaos_bundle/chaosframework/chaos-distrib-i686-linux26/fileProva.txt";
//    stream<<filename.c_str();
//    std::string newfilename = stream.str();
//    
//    myfile.open(newfilename.c_str(),std::ios::out);
//    if(myfile.is_open()){
//        // Descrizione testo effettuato
//        myfile<<"Intervallo di tempo impiegato dalla procedura di homing a fermarsi: "<<std::endl<<std::endl;
//        myfile<<total_time_interval<<std::endl;
//        myfile.close();
//    }
//    else{
//        std::cout<<"Errore nell'apertura del file"<<std::endl;
//    }
}

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
    int resp;
    std::string desc1;
    std::string desc2;
    uint64_t alarms;
    int state;
    
    gettimeofday(&startTimeForMotor1,NULL);
      
        while(total_time_interval<=duration){
            
//            if((resp=OBJ->getState(axisID,&state, descStr))<0){
//                DERR("************** Error returned by getState operation, code error %d **************",resp);
//                sleep(10);
//                //* errPtr = -5;
//            }
//            DPRINT("************** State of axisID 14 partita: %s **************",descStr.c_str());

            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &position_mm_encoder))<0){
                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
                sleep(10);
                //* errPtr = -5;
            }

            //usleep(5000);
            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &position_mm_counter))<0){
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
            
            DPRINT("************** Position encoder of axisID 14: %4.13f **************",position_mm_encoder);
            DPRINT("************** Position counter of axisID 14: %4.13f  **************",position_mm_counter);
            //DPRINT("************** State of axisID 14: %s  **************",desc1.c_str());
            DPRINT("************** Alarms of axisID 14: %s  **************",desc2.c_str());
            DPRINT("************** Code Alarms of axisID 14: %u **************",alarms);
            
            gettimeofday(&endTimeForMotor1,NULL);
            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);

            DPRINT("total_time_interval: %f",total_time_interval);
            
            usleep(1000); // lettura ogni secondo...
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

int procedura(common::actuators::AbstractActuator *OBJ,int numSeq){
    
    DPRINT("************** Procedura n. %d starting **************",numSeq);
    int axisID = 14;

        // MOVIMENTAZIONE ASSE 14
        if(OBJ->poweron(axisID,1)<0){
            DERR("**************Error returned by poweron operation **************");
            sleep(10);
            //* errPtr = -5;
        }
        
        if(OBJ->setParameter(axisID,"speed","200")<0){
            DERR("************** Error setparameter **************");
            sleep(10);
            //* errPtr = -5;
        }
        DPRINT("SETPPARAMETER OK");
        //sleep(35);
        
        //DPRINT("************** Prima movimentazione asse 14, 8 settembre 2014 **************");
        int resp;
        //sleep(5);

//        if((resp=OBJ->moveAbsoluteMillimeters(axisID,1))<0){
//            DERR("************** Error returned by movement operation, code error %d **************",resp);
//            sleep(10);
//            //* errPtr = -5;
//        }

        //sleep(20); // ********** Diamo tempo al thread di movimentazione relativa di completare l'operazione *************

//        if((resp=OBJ->stopMotion(axisID))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//        }
//
//        sleep(10);
//
//        DPRINT("togliamo corrente");
//        if(OBJ->poweron(axisID,0)<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//        }
//
//        sleep(10);
//
//        DPRINT("************** Seconda movimentazione asse 14 **************");
//        if((resp=OBJ->moveAbsoluteMillimeters(axisID,3))<0){
//            DERR("************** Error returned by movement operation, code error %d **************",resp);
//            sleep(10);
//            //* errPtr = -5;
//        }
//
//        sleep(20); // ********** Diamo tempo al thread di movimentazione relativa di completare l'operazione *************
//
//        int state;
//        std::string descStr;

//        struct timeval startTimeForMotor1,endTimeForMotor1;

//        double total_time_interval=0;
//        double duration = 15;
//        double position_mm_encoder;
//        double position_mm_counter;
        
        

//        DPRINT("Stopping displacement");
//        if((resp=OBJ->stopMotion(axisID))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//        }
//
//        sleep(10);
//
//        DPRINT("Togliamo corrente");
//        if(OBJ->poweron(axisID,0)<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//        }
//
//        sleep(10);
//
//        DPRINT("Riinseriamo corrente");
//        if(OBJ->poweron(axisID,1)<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//        }
//
        DPRINT("************** Fra cinque secondi partirà la movimentazione relativa **************");
        sleep(5);

//        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &deltaPosition_mm))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);

        if((resp=OBJ->moveRelativeMillimeters(axisID,10))<0){
            DERR("************** Error returned by movement operation, code error %d **************",resp);
            sleep(10);
            //* errPtr = -5;
        }
        
//        DPRINT("************ Procedura di homing terminata con successo. Segue la lettura dei dati ***************");
//        uint64_t alarms;
//        std::string desc2;
        
        double durationChecking = 60; // secondi
        
        pthread_t th1;
        checkData hd2;
        hd2.axisID=axisID;
        hd2.duration=durationChecking;
        hd2.obj=OBJ;
        
        pthread_create(&th1,NULL,checkProcedures,(void*)&hd2);
        pthread_join(th1,NULL);
        
        //checkProcedures(axisID, durationChecking,OBJ); // secondo parametro: durata intervallo di tempo dedicato al checking
        
//        gettimeofday(&startTimeForMotor1,NULL);
//        
//        while(total_time_interval<=duration){
//            
////            if((resp=OBJ->getState(axisID,&state, descStr))<0){
////                DERR("************** Error returned by getState operation, code error %d **************",resp);
////                sleep(10);
////                //* errPtr = -5;
////            }
////            DPRINT("************** State of axisID 14 partita: %s **************",descStr.c_str());
//
//            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &position_mm_encoder))<0){
//                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//                sleep(10);
//                //* errPtr = -5;
//            }
//
//            //usleep(5000);
//            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &position_mm_counter))<0){
//                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//                sleep(10);
//                //* errPtr = -5;
//            }
//            
//            if((resp=OBJ->getAlarms(axisID,&alarms,desc2))<0){
//                DERR("************** Error reading alarms ***************");
//            }
//            
//            DPRINT("************** Position encoder of axisID 14: %4.13f **************",position_mm_encoder);
//            DPRINT("************** Position counter of axisID 14: %4.13f  **************",position_mm_counter);
//            DPRINT("************** Alarms of axisID 14: %s  **************",desc2.c_str());
//            
//            gettimeofday(&endTimeForMotor1,NULL);
//            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
//
//            DPRINT("total_time_interval: %f",total_time_interval);
//            
//            usleep(1000000); // lettura ogni secondo...
////
//        }
        
//        sleep(10); // ********** Diamo tempo al thread di movimentazione relativa di completare l'operazione *************

//        if((resp=OBJ->stopMotion(axisID))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//        }

//        DPRINT("Position Reading");
//        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &deltaPosition_mm))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//                //* errPtr = -5;
//        }
//
//        DPRINT("************** Position ENCODER of axisID 14: %f mm **************",deltaPosition_mm);
//
//        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &deltaPosition_mm))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
//                //* errPtr = -5;
//        }

        //DPRINT("************** Position COUNTER of axisID 14: %f mm **************",position_mm);

        // HOMING OPERATION
//        int respHoming=1; // Operazione di homing non conclusa
//        int numHoming = 1;
//        
//        DPRINT("************** Homing operation starting for axis 14. Total homing procedure = %d **************", numHoming);
//        sleep(10);
//        
//        for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
//            // con successo o insuccesso
//            DPRINT("*************Procedura di homing n. %d iniziata*************",i);
//            while(respHoming){ // Finche' la procedura di homing non e' completata con successo
//                respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::homing2); // Il parametro in ingresso alla funzione non e' piu letto
//                usleep(1000); // FREQUENZA DI 100 ms
//                if(respHoming<0){
//                    DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
//                    break;
//                }
//            }
//            if(respHoming==0){
//                DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
//            }
//            respHoming = 1;
//            usleep(5000000);
//        }
       
        DPRINT("************** Homing operation starting. ***************");
        sleep(5);
        pthread_t th2;
        homingData hd;
        hd.a=1;
        hd.b=1;
        hd.obj=OBJ;
        hd.c=axisID;
        pthread_create(&th2,NULL,homingProcedures,(void*)&hd);
        //homingProcedures(1,1,OBJ,axisID); // Secondo parametro> numero di volte in cui vuoi eseguire la procedura di homing
          
//        DPRINT("************** Visione andamento procedura di homing per 90 secondi **************");
//        sleep(5);
        durationChecking=120;
        pthread_t th3;
        hd2.axisID=axisID;
        hd2.duration = durationChecking;
        hd2.obj=OBJ;
        
        pthread_create(&th3,NULL,checkProcedures,(void*)&hd2);
       
        //checkProcedures(axisID, durationChecking,OBJ);
        
//        DPRINT("************** Fra cinque secondi partirà la movimentazione relativa 2 **************");
//        sleep(5);
        
        DPRINT("************** Fra 2 secondi tentativo di stoppare la procedura di homing **************");
        //sleep(10);
    
//        pthread_t th4;
//        stopMotionStruct strct;
//        strct.axisID=axisID;
//        strct.obj=OBJ;
//        
//        sleep(15);
//        pthread_create(&th4,NULL,stopMotionProcedure,(void*)&strct);
//        pthread_join(th4,NULL);
//        pthread_join(th3,NULL);
//        pthread_join(th2,NULL);
        
        pthread_t th7;
        resetAlarmsStruct strct2;
        strct2.axisID=axisID;
        strct2.obj=OBJ;
        
        sleep(15);
        pthread_create(&th7,NULL,resetAlarmsProcedure,(void*)&strct2);
        pthread_join(th7,NULL);
        //pthread_join(th3,NULL);
        //pthread_join(th2,NULL);
        
//        
        //sleep(60);
  
//        if((resp=OBJ->stopMotion(axisID))<0){
//            DERR("************** Error returned by movement operation, code error %d **************",resp);
//            sleep(10);
//            //* errPtr = -5;
//        }
        
//        DPRINT("************** Continuiamo a fare il checking sulla procedura di homing per 10 secondi**************");
//        sleep(2);
//        pthread_t th5;
//        hd2.duration=10;
//        pthread_create(&th5,NULL,checkProcedures,(void*)&hd2);
//        
//        pthread_join(th2,NULL);
//        pthread_join(th5,NULL);
//        
//        DPRINT("************** Sequenza operazioni n. %d completata **************",numSeq);
//        sleep(5);
//        
        
//        DPRINT("************** lANCIO OPERAZIONI DI HOMING SENZA BLOCCO **************");
//        sleep(5);
//        
//        hd.a=1;
//        hd.b=1;
//        hd.obj=OBJ;
//        hd.c=axisID;
//        pthread_create(&th2,NULL,homingProcedures,(void*)&hd);
//        
//        hd2.duration=180;
//        pthread_create(&th3,NULL,checkProcedures,(void*)&hd2);
//        
//        pthread_join(th2,NULL);
//        pthread_join(th3,NULL);
   
        return 0;
}

void* function1(void* str){

    int ret;
    char* strInit =(char*)str;

    common::actuators::AbstractActuator *OBJ = new ActuatorTechnoSoft();

    // INIZIALIZZAZIONE CANALE
    if((ret=OBJ->init((void*)strInit))!=0){
        DERR("*************Cannot init channel. In fact the value returned is %d****************",ret);

    }
    //PROCEDURA DI CONFIGURAZIONE MOTORI, SEMPRE SULLO STESSO OGGETTO !!!

    std::string strConfig14 = "14,../common/actuators/models/technosoft/conf/1setup001.t.zip";
    if((ret=OBJ->configAxis((void*)strConfig14.c_str()))!=0){
        DERR("*************Cannot configure axis. In fact the value returned is %d****************",ret);
    }
//    std::string strConfig15 = "15,../common/actuators/models/technosoft/conf/1setup001.t.zip";
//    if((ret=OBJ->configAxis((void*)strConfig15.c_str()))!=0){
//        DERR("*************Cannot configure axis. In fact the value returned is %d****************",ret);
//    }
    else{ // Invio comandi ai motori (axisID = 14)
        
        int numVolteProcedura=1;
        for(int i=1;i<=numVolteProcedura;i++){
            procedura(OBJ,i);
        }
        
        //int axisID=14;
        
//        DPRINT("************** lANCIO OPERAZIONI DI HOMING SENZA BLOCCO **************");
//        sleep(5);
//        
//        pthread_t th2;
//        homingData hd;
//        hd.a=1;
//        hd.b=1;
//        hd.obj=OBJ;
//        hd.c=axisID;
//        pthread_create(&th2,NULL,homingProcedures,(void*)&hd);
        //homingProcedures(1,1,OBJ,axisID); // Secondo parametro> numero di volte in cui vuoi eseguire la procedura di homing
//          
          //DPRINT("************** Visione andamento procedura di homing per 180 secondi **************");
        
//        double durationChecking = 180; // secondi
//        
//        checkData hd2;
//        pthread_t th3;
//        hd2.axisID=axisID;
//        hd2.duration = durationChecking;
//        hd2.obj=OBJ;
//        
//        pthread_create(&th3,NULL,checkProcedures,(void*)&hd2);
//        pthread_join(th2,NULL);
//        pthread_join(th3,NULL);
        
//        int axisID = 14;
//
//        // MOVIMENTAZIONE ASSE 14
//        if(OBJ->poweron(axisID,1)<0){
//            DERR("**************Error returned by poweron operation **************");
//            sleep(10);
//            //* errPtr = -5;
//        }
//        
//        if(OBJ->setParameter(axisID,"speed","200")<0){
//            DERR("************** Error setparameter **************");
//            sleep(10);
//            //* errPtr = -5;
//        }
//        DPRINT("SETPPARAMETER OK");
//        //sleep(35);
//        
//        //DPRINT("************** Prima movimentazione asse 14, 8 settembre 2014 **************");
//        int resp;
//        //sleep(5);
//
////        if((resp=OBJ->moveAbsoluteMillimeters(axisID,1))<0){
////            DERR("************** Error returned by movement operation, code error %d **************",resp);
////            sleep(10);
////            //* errPtr = -5;
////        }
//
//        //sleep(20); // ********** Diamo tempo al thread di movimentazione relativa di completare l'operazione *************
//
////        if((resp=OBJ->stopMotion(axisID))<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////        }
////
////        sleep(10);
////
////        DPRINT("togliamo corrente");
////        if(OBJ->poweron(axisID,0)<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////        }
////
////        sleep(10);
////
////        DPRINT("************** Seconda movimentazione asse 14 **************");
////        if((resp=OBJ->moveAbsoluteMillimeters(axisID,3))<0){
////            DERR("************** Error returned by movement operation, code error %d **************",resp);
////            sleep(10);
////            //* errPtr = -5;
////        }
////
////        sleep(20); // ********** Diamo tempo al thread di movimentazione relativa di completare l'operazione *************
////
////        int state;
////        std::string descStr;
//
////        struct timeval startTimeForMotor1,endTimeForMotor1;
//
////        double total_time_interval=0;
////        double duration = 15;
////        double position_mm_encoder;
////        double position_mm_counter;
//        
//        
//
////        DPRINT("Stopping displacement");
////        if((resp=OBJ->stopMotion(axisID))<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////        }
////
////        sleep(10);
////
////        DPRINT("Togliamo corrente");
////        if(OBJ->poweron(axisID,0)<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////        }
////
////        sleep(10);
////
////        DPRINT("Riinseriamo corrente");
////        if(OBJ->poweron(axisID,1)<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////        }
////
//        DPRINT("************** Fra cinque secondi partirà la movimentazione relativa **************");
//        sleep(5);
//
////        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &deltaPosition_mm))<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
//
//        if((resp=OBJ->moveRelativeMillimeters(axisID,10))<0){
//            DERR("************** Error returned by movement operation, code error %d **************",resp);
//            sleep(10);
//            //* errPtr = -5;
//        }
//        
////        DPRINT("************ Procedura di homing terminata con successo. Segue la lettura dei dati ***************");
////        uint64_t alarms;
////        std::string desc2;
//        
//        double durationChecking = 60; // secondi
//        
//        pthread_t th1;
//        checkData hd2;
//        hd2.axisID=axisID;
//        hd2.duration=durationChecking;
//        hd2.obj=OBJ;
//        
//        pthread_create(&th1,NULL,checkProcedures,(void*)&hd2);
//        pthread_join(th1,NULL);
//        
//        //checkProcedures(axisID, durationChecking,OBJ); // secondo parametro: durata intervallo di tempo dedicato al checking
//        
////        gettimeofday(&startTimeForMotor1,NULL);
////        
////        while(total_time_interval<=duration){
////            
//////            if((resp=OBJ->getState(axisID,&state, descStr))<0){
//////                DERR("************** Error returned by getState operation, code error %d **************",resp);
//////                sleep(10);
//////                //* errPtr = -5;
//////            }
//////            DPRINT("************** State of axisID 14 partita: %s **************",descStr.c_str());
////
////            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &position_mm_encoder))<0){
////                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////                sleep(10);
////                //* errPtr = -5;
////            }
////
////            //usleep(5000);
////            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &position_mm_counter))<0){
////                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////                sleep(10);
////                //* errPtr = -5;
////            }
////            
////            if((resp=OBJ->getAlarms(axisID,&alarms,desc2))<0){
////                DERR("************** Error reading alarms ***************");
////            }
////            
////            DPRINT("************** Position encoder of axisID 14: %4.13f **************",position_mm_encoder);
////            DPRINT("************** Position counter of axisID 14: %4.13f  **************",position_mm_counter);
////            DPRINT("************** Alarms of axisID 14: %s  **************",desc2.c_str());
////            
////            gettimeofday(&endTimeForMotor1,NULL);
////            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
////
////            DPRINT("total_time_interval: %f",total_time_interval);
////            
////            usleep(1000000); // lettura ogni secondo...
//////
////        }
//        
////        sleep(10); // ********** Diamo tempo al thread di movimentazione relativa di completare l'operazione *************
//
////        if((resp=OBJ->stopMotion(axisID))<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////        }
//
////        DPRINT("Position Reading");
////        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &deltaPosition_mm))<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////                //* errPtr = -5;
////        }
////
////        DPRINT("************** Position ENCODER of axisID 14: %f mm **************",deltaPosition_mm);
////
////        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &deltaPosition_mm))<0){
////            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
////            sleep(10);
////                //* errPtr = -5;
////        }
//
//        //DPRINT("************** Position COUNTER of axisID 14: %f mm **************",position_mm);
//
//        // HOMING OPERATION
////        int respHoming=1; // Operazione di homing non conclusa
////        int numHoming = 1;
////        
////        DPRINT("************** Homing operation starting for axis 14. Total homing procedure = %d **************", numHoming);
////        sleep(10);
////        
////        for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
////            // con successo o insuccesso
////            DPRINT("*************Procedura di homing n. %d iniziata*************",i);
////            while(respHoming){ // Finche' la procedura di homing non e' completata con successo
////                respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::homing2); // Il parametro in ingresso alla funzione non e' piu letto
////                usleep(1000); // FREQUENZA DI 100 ms
////                if(respHoming<0){
////                    DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
////                    break;
////                }
////            }
////            if(respHoming==0){
////                DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
////            }
////            respHoming = 1;
////            usleep(5000000);
////        }
//       
//        DPRINT("************** Homing operation starting. ***************");
//        sleep(5);
//        pthread_t th2;
//        homingData hd;
//        hd.a=1;
//        hd.b=1;
//        hd.obj=OBJ;
//        hd.c=axisID;
//        pthread_create(&th2,NULL,homingProcedures,(void*)&hd);
//        //homingProcedures(1,1,OBJ,axisID); // Secondo parametro> numero di volte in cui vuoi eseguire la procedura di homing
//          
//        DPRINT("************** Visione andamento procedura di homing per 30 secondi **************");
//        
//        durationChecking=30;
//        pthread_t th3;
//        hd2.axisID=axisID;
//        hd2.duration = durationChecking;
//        hd2.obj=OBJ;
//        
//        pthread_create(&th3,NULL,checkProcedures,(void*)&hd2);
//        
//        //checkProcedures(axisID, durationChecking,OBJ);
//        
////        DPRINT("************** Fra cinque secondi partirà la movimentazione relativa 2 **************");
////        sleep(5);
//        
//        DPRINT("************** Fra 2 secondi tentativo di stoppare la procedura di homing **************");
//        //sleep(10);
//    
//        pthread_t th4;
//        stopMotionStruct strct;
//        strct.axisID=axisID;
//        strct.obj=OBJ;
//        
//        sleep(15);
//        pthread_create(&th4,NULL,stopMotionProcedure,(void*)&strct);
//        pthread_join(th4,NULL);
//        pthread_join(th3,NULL);
//        
//        //sleep(60);
//  
////        if((resp=OBJ->stopMotion(axisID))<0){
////            DERR("************** Error returned by movement operation, code error %d **************",resp);
////            sleep(10);
////            //* errPtr = -5;
////        }
//        
//        DPRINT("************** Continuiamo a fare il checking sulla procedura di homing  per 10 secondi**************");
//        sleep(2);
//        pthread_t th5;
//        hd2.duration=10;
//        pthread_create(&th5,NULL,checkProcedures,(void*)&hd2);
//        
//        pthread_join(th2,NULL);
//        pthread_join(th5,NULL);
//        
//        
//        // FINE PRIMA ESECUZIONE DEL THREAD
//        DPRINT("************** Seconda nuova movimentazione in avanti fra 2 secondi **************");
        
        
        
        
        //checkProcedures(axisID, durationChecking,OBJ);
        
 
//        if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &deltaPosition_mm))<0){
//            DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//            sleep(10);
        
//        if((resp=OBJ->moveAbsoluteMillimeters(axisID,5))<0){
//            DERR("************** Error returned by movement operation, code error %d **************",resp);
//            sleep(10);
//            //* errPtr = -5;
//        }
//        
//        durationChecking=30;
//        checkProcedures(axisID, durationChecking,OBJ);
//        
//        DPRINT("************** Homing operation 2. ***************");
//        sleep(5);
//        homingProcedures(1,1,OBJ,axisID); // Secondo parametro> numero di volte in cui vuoi eseguire la procedura di homing
//        
//        durationChecking=15;
//        checkProcedures(axisID, durationChecking,OBJ);
        
//        gettimeofday(&startTimeForMotor1,NULL);
//        while(total_time_interval<=duration){
//
////            if((resp=OBJ->getState(axisID,&state, descStr))<0){
////                DERR("************** Error returned by getState operation, code error %d **************",resp);
////                sleep(10);
////                //* errPtr = -5;
////            }
////            DPRINT("************** State of axisID 14 partita: %s **************",descStr.c_str());
//
//            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_ENCODER, &position_mm_encoder))<0){
//                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//                sleep(10);
//                //* errPtr = -5;
//            }
//
//            //usleep(5000);
//            if((resp=OBJ->getPosition(axisID,common::actuators::AbstractActuator::READ_COUNTER, &position_mm_counter))<0){
//                DERR("************** Error returned by getPosition operation, code error %d **************",resp);
//                sleep(10);
//                //* errPtr = -5;
//            }
//            
//            if((resp=OBJ->getAlarms(axisID,&alarms,desc2))<0){
//                DERR("************** Error reading alarms ***************");
//            }
//
//            DPRINT("************** Position encoder of axisID 14: %4.13f **************",position_mm_encoder);
//            DPRINT("************** Position counter of axisID 14: %4.13f  **************",position_mm_counter);
//            DPRINT("************** Alarms of axisID 14: %s  **************",desc2.c_str());
//            
//            gettimeofday(&endTimeForMotor1,NULL);
//            total_time_interval = ((double)endTimeForMotor1.tv_sec+(double)endTimeForMotor1.tv_usec/1000000.0)-((double)startTimeForMotor1.tv_sec+(double)startTimeForMotor1.tv_usec/1000000.0);
//
//            usleep(1000000); // lettura ogni millisecondo..
////
//        }
        

//        DPRINT("************** Movimentazione asse 14 partita. Rimarro' in attesa 30 s per far finire la movimentazione **************");
//        sleep(30);

//        // MOVIMENTAZIONE ASSE 15
//        axisID = 15;
//        if(OBJ->poweron(axisID,1)<0){
//            DERR("**************Error returned by stop motion operation **************");
//            //* errPtr = -5;
//        }
//
//        DPRINT("************** Movimentazione asse 15 **************");
//        if(OBJ->moveRelativeMillimeters(axisID,10)<0){
//            DERR("************** Error returned by movement operation **************");
//            //* errPtr = -5;
//        }
//        DPRINT("************** Movimentazione asse 15 partita. Rimarro' in attesa 30 s per far finire la movimentazione **************");
//        sleep(30);
//
//        // MOVIMENTAZIONE ASSE 14
//        axisID = 14;
//
//        DPRINT("************** Movimentazione asse 14 **************");
//        if(OBJ->moveRelativeMillimeters(axisID,10)<0){
//            DERR("************** Error returned by movement operation **************");
//            //* errPtr = -5;
//        }
//        DPRINT("************** Movimentazione asse 14 partita. Rimarro' in attesa 30 s per far finire la movimentazione **************");
//        sleep(30);
//
//     // HOMING OPERATION
//        int respHoming=1; // Operazione di homing non conclusa
//        int numHoming = 3;
//
//        DPRINT("************** Homing operation starting for axis 14. Total homing procedure = %d **************", numHoming);
//        for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
//            // con successo o insuccesso
//            DPRINT("*************Procedura di homing n. %d iniziata*************",i);
//            while(respHoming){ // Finche' la procedura di homing non e' completata con successo
//                respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::defaultHoming); // Il parametro in ingresso alla funzione non e' piu letto
//                usleep(100000); // FREQUENZA DI 1,5 ms
//                if(respHoming<0){
//                    DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
//                    break;
//                }
//          
                  
//            if(respHoming==0){
//                DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
//            }
//            respHoming = 1;
//            usleep(5000000);
//        }
//
//        DPRINT("************** Sleep di 10 secondi alla fine di tutte le operazioni di homing**************");
//        sleep(10);
//
//        // HOMING OPERATION
//        respHoming=1; // Operazione di homing non conclusa
//        numHoming = 3;
//        axisID = 15;
//        DPRINT("************** Homing operation starting for axis 15. Total homing procedure = %d **************", numHoming);
//        for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
//            // con successo o insuccesso
//            DPRINT("*************Procedura di homing n. %d iniziata*************",i);
//            while(respHoming){ // Finche' la procedura di homing non e' completata con successo
//                respHoming = OBJ->homing(axisID,common::actuators::AbstractActuator::defaultHoming); // Il parametro in ingresso alla funzione non e' piu letto
//                usleep(100000); // FREQUENZA DI 1,5 ms
//                if(respHoming<0){
//                    DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
//                    break;
//                }
//            }
//            if(respHoming==0){
//                DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
//            }
//            respHoming = 1;
//            usleep(5000000);
//        }
//
//        DPRINT("************** Sleep di 10 secondi alla fine di tutte le operazioni di homing**************");
//        sleep(10);
//
//        axisID = 14;
//        if(OBJ->poweron(axisID,0)<0){
//            DERR("**************Error returned by stop motion operation **************");
//            //* errPtr = -5;
//        }
//        axisID = 15;
//        if(OBJ->poweron(axisID,0)<0){
//            DERR("**************Error returned by stop motion operation **************");
//            //* errPtr = -5;
//        }
//
//        DPRINT("************** deinit Asse 14 **************");
//        OBJ->deinit(14);
//
//        DPRINT("************** deinit Asse 15 **************");
//        OBJ->deinit(15);
//
//        DPRINT("************** delete ActuatorTechnoSoft OBJ **************");
//        delete OBJ;

        return 0;
    }
}


//int funzioneProva(ActuatorTechnoSoft slit){
//    DPRINT("************** Movement operation starting from function **************");
//    if(slit.moveRelativeMillimeters(10)<0){
//            DERR("**************Error returned by movement operation **************");
//            //* errPtr = -5;
//            return -1;
//    }
//    sleep(25);
//    DPRINT("************** Movement absolute operation starting from function **************");
//    if(slit.moveAbsoluteMillimeters(27)<0){
//        DERR("**************Error returned by movement operation **************");
//        //* errPtr = -5;
//    }
//    sleep(180);
//    DPRINT("************** Homing operation starting from function **************");
//    int respHoming=1; // Operazione di homing non conclusa
//    int numHoming = 3;
//
//    for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
//        // con successo o insuccesso
//        DPRINT("*************Procedura di homing n. %d iniziata*************",i);
//        while(respHoming){ // Finche' la procedura di homing non e' completata con successo
//            respHoming = slit.homing(common::actuators::AbstractActuator::defaultHoming); // Il parametro in ingresso alla funzione non e' piu letto
//            usleep(100000); // FREQUENZA DI 1,5 ms
//            if(respHoming<0){
//                DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
//                break;
//            }
//         }
//        if(respHoming==0){
//            DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
//        }
//        respHoming = 1;
//        usleep(5000000);
//    }
//    return 0;
//}

void* function2(void* str){

//    //int* errPtr = new int; // Nessun errore..
//    int ret;
//
//    char* strInit =(char*)str;
////    ActuatorTechnoSoft mySlit1; // ATTENZIONE: NON E' STATA GESTITA L'ECCEZIONE BAD_ALLOC
////    DPRINT("%s",strInit);
////
////    double pos2;
////    int axis2;
////    const char *dev2,*conf2;
////    char sinit2[256];
////    // Inizializzazione parametri ASSE 2
////    dev2="/dev/ttyr1d";        // [string], <dev/tty>
////    conf2="../common/actuators/models/technosoft/conf/1setup001.t.zip";       // [string], <technosoft configuration>
////    axis2=15; // [int], <axis>
////    sprintf(sinit2,"%s,myslit2,%s,%d",dev2,conf2,axis2);
//    ActuatorTechnoSoft mySlit2;
//    //&& ((ret=mySlit2.init((void*)strInit))!=0)
//
//    sleep(5);
//    DPRINT("************* Sono la function 2. Inizio fase di inizializzazione dopo 5 s di attesa ****************");
//    if((ret=mySlit2.init((void*)strInit))!=0){
//        DERR("*************Cannot init axis. In fact the value returned is %d****************",ret);
//        //* errPtr = -1;
//        //delete mySlit1;
//        //mySlit1 = NULL;
//    }
////    if((ret=mySlit2.init((void*)sinit2))!=0){
////        DERR("*************Cannot init axis. In fact the value returned is %d****************",ret);
////        //* errPtr = -1;
////        //delete mySlit1;
////        //mySlit1 = NULL;
////    }
//    else{
//
////        // Lettura stato
////        std::string desc1;
////        int status;
////        if(mySlit.getState(&status,desc1)<0){
////	DERR("**************Axis: error at first reading status**************");
////        //* errPtr = -2;
////        }
////        DPRINT("************** State before move relative: %d, %s **************",status,desc1.c_str());
////
////        uint64_t alarms;
////        std::string desc2;
////        if(mySlit1.getAlarms(&alarms,desc2)<0){
////	DERR("************** Error at first reading alarms **************");
////        //* errPtr = -2;
////        }
////        DPRINT("************** Alarms before move relative: %lu, %s **************",alarms,desc2.c_str());
////        std::cout<<desc2<<std::endl;
////
////        // Reset alarms
////        int respAlarms;
////        if((respAlarms=mySlit1.resetAlarms(0))<0){
////            DERR("**************Error setting alarms %d **************\n",respAlarms);
////        }
////
////        if(mySlit1.getAlarms(&alarms,desc2)<0){
////	DERR("************** Reading alarms after reset **************");
////        //* errPtr = -2;
////        }
////        DPRINT("************** Reading alarms after reset: %lu, %s **************",alarms,desc2.c_str());
////        std::cout<<desc2<<std::endl;
////
////        // Lettura posizione tramite encoder e counter
//        double rpos=-1000000,rpos1=-1000000;
////        if(mySlit1.getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
////            //* errPtr = -3;
////        }
////        if(mySlit1.getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
////            DERR("************** Error at first position reading by encoder **************");
////            //* errPtr = -4;
////        }
////        DPRINT("**************Current position encoder %f, before move relative **************",rpos);
////        DPRINT("**************Current position counter %f, before move relative **************",rpos1);
//
//
//        if(mySlit2.poweron(1)<0){
//            DERR("**************Error returned by stop motion operation **************");
//            //* errPtr = -5;
//        }
////
////        DPRINT("************** Movement operation motor 1 **************");
////        if(mySlit1.moveRelativeMillimeters(10)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////        sleep(25);
//
////        if(mySlit2.getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
////            DERR("************** Error at first position reading by encoder **************");
////            //* errPtr = -4;
////        }
////        DPRINT("**************Current position counter %f, before move relative **************",rpos1);
////
////        DPRINT("************** Movement operation motor 2 **************");
//        DPRINT("************** Movement operation motor thread 2 **************");
//        if(mySlit2.moveRelativeMillimeters(10)<0){
//            DERR("************** Error returned by movement operation **************");
//            //* errPtr = -5;
//        }
//
////        DPRINT("%s",sinit1);
////        DPRINT("************** Sono la function 1. Movimentazione partita. Rimarro' in attesa 400 s per far finire la movimentazione **************");
////        sleep(300);
////
////        mySlit2.setParameter("  SPEed       ","   127    ");
////        if(mySlit2.moveRelativeMillimeters(17)<0){
////            DERR("************** Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////        sleep(100);
//
////        mySlit2=mySlit1;
////        DPRINT("************** Operazione di assegnamento eseguita **************");
////        if(mySlit2.moveRelativeMillimeters(10)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////        sleep(25);
//
////        if(funzioneProva(mySlit1)<0){
////            DERR("**************Error returned by movement operation from function **************");
////            //* errPtr = -5;
////        }
//
////        DPRINT("************** Movement operation after functionProva **************");
////        if(mySlit1.moveRelativeMillimeters(10)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////        sleep(25);
//
////
////        if(mySlit1.setSpeed(399.0)<0){
////            DERR("************** Error at setSpeed **************");
////            //* errPtr = -4;
////        }
//
////        if(mySlit1->setMaxSpeed(501.0)<0){
////            DERR("************** Error at MaxSpeed **************");
////            //* errPtr = -4;
////        }
//
////        if(mySlit1.setAcceleration(0.5)<0){
////            DERR("************** Error at setAcceleration **************");
////            //* errPtr = -4;
////        }
//
////        if(mySlit1->setMaxAcceleration(1.7)<0){
////            DERR("************** Error at MaxAcceleration **************");
////            //* errPtr = -4;
////        }
//
////        if(mySlit1.setAdditive(0)<0){
////            DERR("************** Error at setAdditive **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1.setMovement(1)<0){
////            DERR("************** Error at movement **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1.setReferenceBase(1)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->sethighSpeedHoming(10.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setMaxhighSpeedHoming(16)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setlowSpeedHoming(1.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setMaxlowSpeedHoming(3.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setAccelerationHoming(0.4)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setMaxAccelerationHoming(0.7)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setAdditiveHoming(0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setMovementHoming(1)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setReferenceBaseHoming(1)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setEncoderLines(800.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setConst_mult_technsoft(256.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setSteps_per_rounds(200.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setN_rounds(20.0)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setLinear_movement_per_n_rounds(1.5)<0){
////            DERR("************** Error at referenceBase **************");
////            //* errPtr = -4;
////        }
//
////        DPRINT("************** Movement operation starting 2 **************");
////        if(mySlit1.moveRelativeMillimeters(10)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////
////        sleep(25);
////
////        // ATTENZIONE: PRIMA DELLA DEALLOCAZIONE DOVRANNO ESSERE ESEGUITE IN FUTURO QUESTE DUE FUNZIONI
////        if(mySlit1.stopMotion()<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////        if(mySlit1.poweron(0)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
//
//
////        if(mySlit1->moveAbsoluteMillimeters(27)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////
////        sleep(25);
//
//
//
////
////        if(mySlit1->setTrapezoidalProfile(100,0.2,1,0,0)<0){
////            DERR("************** Error setReferenceBase **************");
////        }
////
////        if(mySlit1->moveRelativeMillimeters(10)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -5;
////        }
////        sleep(30);
////    DPRINT("************** 1 done **************");
////
////    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
////        * errPtr = -3;
////    }
////    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
////    	DERR("************** Error at first position reading by encoder **************");
////        * errPtr = -4;
////    }
////    DPRINT("**************Current position encoder %f,after move relative **************",rpos);
////    DPRINT("**************Current position counter %f, after move relative **************",rpos1);
////
//////    if(mySlit1->moveRelativeMillimeters(10)<0){
//////	DERR("**************Error returned by movement operation **************");
//////        * errPtr = -6;
//////    }
//////    sleep(25);
//////    DPRINT("************** 2 done **************");
//////    if(mySlit1->moveRelativeMillimeters(10)<0){
//////	DERR("**************Error returned by movement operation **************");
//////        * errPtr = -7;
//////    }
//////    sleep(25);
//////    DPRINT("************** 3 done **************");
//////    if(mySlit1->moveRelativeMillimeters(10)<0){
//////	DERR("**************Error returned by movement operation **************");
//////        * errPtr = -8;
//////    }
//////    sleep(25); // Attesa completamento movimentazione, in seconds
//////    DPRINT("************** 4 done **************");
//////    if(mySlit1->moveRelativeMillimeters(10)<0){
//////	DERR("**************Error returned by movement operation **************");
//////        * errPtr = -9;
//////    }
//////    sleep(25);
//////    DPRINT("************** 5 done **************");
//////     if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//////        * errPtr = -10;
//////    }
//////    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//////    	DERR("************** Error at first position reading by encoder **************");
//////        * errPtr = -11;
//////    }
//     // Move absolute millimeters homing
////        DPRINT("************** Absolute movement operation starting **************");
////        //double absoluteMovement = rpos+15;
////        if(mySlit1->moveAbsoluteMillimeters(20)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -6;
////        }
////        sleep(60);
//
////        if(mySlit1->setSpeed(100)<0){
////            DERR("************** Error setSpeed **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setAcceleration(0.2)<0){
////            DERR("************** Error setAcceleration **************");
////            //* errPtr = -4;
////        }
////        if(mySlit1->setAdditive(1)<0){
////            DERRhoming2("************** Error setAdditive **************");
////        }
////        if(mySlit1->setMovement(0)<0){
////            DERR("************** Error setMovement **************");
////        }
////        if(mySlit1->setReferenceBase(0)<0){
////            DERR("************** Error setReferenceBase **************");
////        }
//
////        if(mySlit1->setTrapezoidalProfile(100,0.2,1,0,0)<0){
////            DERR("************** Error setReferenceBase **************");
////        }
////
////        if(mySlit1->moveAbsoluteMillimeters(10)<0){
////            DERR("**************Error returned by movement operation **************");
////            //* errPtr = -6;
////        }
////        sleep(60);
////        if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
////            //* errPtr = -7;
////        }
////        if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
////            DERR("************** Error at first position reading by encoder **************");
////            //* errPtr = -8;
////        }
////        DPRINT("**************Current position encoder %f, after absolute movement **************",rpos);
////        DPRINT("**************Current position counter %f, after absolute movement **************",rpos1);
//
////        int respHoming=1; // Operazione di homing non conclusa
////        int numHoming = 3;
////
////        for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
////            // con successo o insuccesso
////            DPRINT("*************Procedura di homing n. %d iniziata*************",i);
////            while(respHoming){ // Finche' la procedura di homing non e' completata con successo
////                respHoming = mySlit1->homing(common::actuators::AbstractActuator::defaultHoming); // Il parametro in ingresso alla funzione non e' piu letto
////                usleep(100000); // FREQUENZA DI 1,5 ms
////                if(respHoming<0){
////                    DERR("***************Procedura di homing n. %d terminata con errore ***************",respHoming);
////                    break;
////                }
////            }
////            if(respHoming==0){
////                DPRINT("************Procedura di homing n. %d terminata con successo ***************",i);
////            }
////            respHoming = 1;
////            usleep(5000000);
////        }
////
////        if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
////            //* errPtr = -7;
////        }
////        if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
////            DERR("************** Error at first position reading by encoder **************");
////            //* errPtr = -8;
////        }
////        DPRINT("**************Current position encoder %f, after homing **************",rpos);
////        DPRINT("**************Current position counter %f, after homing **************",rpos1);
//
//        // Deallochiamo tutto prima che termini il thread
////        delete mySlit1;
////        mySlit1 = NULL;
//        //delete errPtr;
//        //errPtr = NULL;
//
//
////        if(mySlit1.stopMotion()<0){
////            DERR("**************Error returned by stop motion operation **************");
////            //* errPtr = -5;
////        }
////        if(mySlit1.poweron(0)<0){
////            DERR("**************Error returned by stop motion operation **************");
////            //* errPtr = -5;
////        }
////
////        if(mySlit2.stopMotion()<0){
////            DERR("**************Error returned by stop motion operation **************");
////            //* errPtr = -5;
////        }
////        if(mySlit2.poweron(0)<0){
////            DERR("**************Error returned by stop motion operation **************");
////            //* errPtr = -5;
////        }
//        //sleep(500);
//        DPRINT("Thread 2 terminato");
//        return 0;
//    }
//    //sleep(5);
//    //pthread_exit((void*)errPtr);

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
//    pthread_join(th2,NULL);

    //int res1 = *((int*)(resp_th_1));
    //int res2 = *((int*)(resp_th_2));

    //DPRINT("**************Esito terminazione thread 1: %d**************",res1);
    //DPRINT("**************Esito terminazione thread 2: %d**************",res2);

//    // Inizializzazione ASSE 2
//    mySlit2 = new ActuatorTechnoSoft(); // ATTENZIONE: NON E' STATA GESTITA L'ECCEZIONE BAD_ALLOC
//
//    if((ret=mySlit2->init((void*)sinit2))!=0){
//        DERR("*************Cannot init axis %d. In fact the value returned is %d ****************",axis2,ret);
//        return -1;
//    }
//    DPRINT("*************Axis %d initialized!****************",axis2);
//
//    // Due init CONSECUTIVE gestiscono correttamente lo scenario multiasse
//
//    // Lettura stato
//    std::string desc;
//    if(mySlit1->getState(&status,desc)<0){
//	DERR("**************Axis %d: error at first reading status**************",axis1);
//        return -2;
//    }
//    DPRINT("**************Axis %d: first reading status: %d, %s **************\n",axis1,status,desc.c_str());
//
//    // Lettura posizione tramite encoder e counter
//    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//	DERR("**************Axis %d: error at first position reading by encoder **************");
//        return -3;
//    }
//    //DPRINT("************** Current position encoder %f, before movement **************",rpos);
//
//    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//    	fprintf(stderr,"**************Error at first position reading by counter **************\n");
//        return -4;
//    }
//    DPRINT("**************Axis %d: Current position encoder %f, before move relative **************",axis1,rpos);
//    DPRINT("**************Axis %d: Current position counter %f, before move relative **************",axis1,rpos1);
//
////    std::string version;
////    if(mySlit1->getSWVersion(version)<0){
////    	DERR("**************Axis %d: Error at fgetSWVersion command **************",axis1);
////        return -5;
////    }
////    DPRINT("************** Axis %d: Firmware version: %s **************",version.c_str(),axis1);
//
//    DPRINT("************** Axis %d: Reset alarms before move relative **************",axis1);
//    int respAlarms;
//    if((respAlarms=mySlit1->resetAlarms(0))<0){
//    	DERR("**************Axis %d: Error setting alarms %d **************\n",axis1,respAlarms);
//        return -5;
//    }
//
//    if(mySlit1->getState(&status,desc)<0){
//	DERR("**************Axis %d: Error at first reading status**************",axis1);
//        return -2;
//    }
//    DERR("**************Axis %d: reading status after setting alarms %d, %s **************\n",axis1,status ,desc.c_str());
//
////    if(mySlit->getState(&status,desc)<0){
////	fprintf(stderr,"**************Error get status after reset fault**************\n");
////        return -6;
////    }
////    DPRINT("************** Reading status after reset alarms: %d, %s **************\n",status ,desc.c_str());
//
//    DPRINT("**************Axis %d: Prima movimentazione di 10 mm **************",axis1);
//    // Spostamento della slitta
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Axis %d: Error returned by movement operation **************",axis1);
//        return -3;
//    }
//    sleep(25);
//    if(mySlit2->moveRelativeMillimeters(10)<0){
//	DERR("**************Axis %d: Error returned by movement operation **************",axis1);
//        return -4;
//    }
//    sleep(25);
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Axis %d: Error returned by movement operation **************",axis1);
//        return -5;
//    }
//    sleep(25);
//    if(mySlit2->moveRelativeMillimeters(10)<0){
//	DERR("**************Axis %d: Error returned by movement operation **************",axis1);
//        return -6;
//    }
//    sleep(25); // Attesa completamento movimentazione, in seconds
//
//    DPRINT("**************Axis %d: Prima movimentazione di 10 mm **************",axis2);
//    if(mySlit2->moveRelativeMillimeters(10)<0){
//	DERR("**************Axis %d: Error returned by movement operation **************",axis2);
//        return -8;
//    }
//
//    DPRINT("**************move relative finished for both motors**************");
//
//    if(mySlit2->getState(&status,desc)<0){
//	DERR("**************Error reading status after move relative**************",axis2);
//        return -8;
//    }
//    DPRINT("**************Axis %d: Reading status %d, %s after move relative**************",axis2,status ,desc.c_str());
//
//    // Lettura posizione tramite encoder e counter
//    if(mySlit2->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//	DERR("**************Axis %d: Error position reading by encoder after move relative **************",axis2);
//        return -9;
//    }
//
//    if(mySlit2->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//    	DERR("**************Error position reading by counter, after move relative **************\n");
//        return -10;
//    }
//    DPRINT("**************Axis %d: Current position encoder %f, after move relative **************",axis2,rpos);
//    DPRINT("**************Axis %d: Current position counter %f, after move relative **************",axis2,rpos1);
//
//

//
//    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//	DERR("**************Axis %d: Error at second position after homing by encoder **************",axis1);
//        return -10;
//    }
//    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//    	DERR("**************Axis %d: Error at second position after homing reading by counter **************",axis1);
//        return -11;
//    }
//    DPRINT("**************Axis %d: Current position encoder %f, after homing **************",axis1,rpos);
//    DPRINT("**************Axis %d: Current position counter %f, after homing **************",axis1,rpos1);
//
//    if(mySlit1->getState(&status,desc)<0){
//	DERR("**************Axis %d: Error at reading status after homing**************",axis1);
//        return -14;
//    }
//    DPRINT("************** Axis %d: Reading status %d, %s after homing **************",axis1,status ,desc.c_str());
//
//    //sleep(10); //Sleep inserito per analizzare la corretta deallocazione delle risorse
//
////    try {
////        if(mySlit1!=NULL){
////            delete mySlit1;
////            DPRINT("Effect of the complete deallocation: possible motion stopped (if channel has been opened); possible electric power interrupted (if channel has been opened); possible opened communication channel closed.");
////        }
////    }
////    catch(StopMotionException e){
////        e.badStopMotionInfo();
////        return -11;
////    }
////    catch(ElectricPowerException e){
////        e.badElectricPowerInfo();
////        return -12;
////    }
//    //sleep(5);
//
    return 0;
}