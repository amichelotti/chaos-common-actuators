#include "iostream"
#include "stdio.h"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "ActuatorTechnoSoft.h"
#include <pthread.h>


using namespace common::actuators::models;
#define USAGE \
  printf("**************Usage is:%s <dev/tty> <technosoft configuration> <axis> <move position in mm>*************\n",argv[0]);


void* function1(void* str){
    
    int* errPtr = new int; // Nessun errore..
    
    char* strInit =(char*)str;
    common::actuators::AbstractActuator* mySlit1 = new ActuatorTechnoSoft(); // ATTENZIONE: NON E' STATA GESTITA L'ECCEZIONE BAD_ALLOC
    
    int ret;
    if((ret=mySlit1->init((void*)strInit))!=0){
        DERR("*************Cannot init axis. In fact the value returned is ****************");
        * errPtr = -1;
    }
    // Lettura stato
    std::string desc;
    int status;
    if(mySlit1->getState(&status,desc)<0){
	DERR("**************Axis: error at first reading status**************");
        * errPtr = -2;
    }
   
    // Lettura posizione tramite encoder e counter
    double rpos=-1000000,rpos1=-1000000;
    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
        * errPtr = -3;
    } 
    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	DERR("************** Error at first position reading by encoder **************");
        * errPtr = -4;
    }
    DPRINT("**************Current position encoder %f, before move relative **************",rpos);
    DPRINT("**************Current position counter %f, before move relative **************",rpos1);
    
    DPRINT("************** Movement operation starting **************");
    if(mySlit1->moveRelativeMillimeters(10)<0){
	DERR("**************Error returned by movement operation **************");
        * errPtr = -5;
    }
    sleep(25);
    DPRINT("************** 1 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -6;
//    }
//    sleep(25);
//    DPRINT("************** 2 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -7;
//    }
//    sleep(25);
//    DPRINT("************** 3 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -8;
//    }
//    sleep(25); // Attesa completamento movimentazione, in seconds
//    DPRINT("************** 4 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -9;
//    }
//    sleep(25);
//    DPRINT("************** 5 done **************");
//     if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//        * errPtr = -10;
//    } 
//    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//    	DERR("************** Error at first position reading by encoder **************");
//        * errPtr = -11;
//    }
    DPRINT("**************Current position encoder %f, before move relative **************",rpos);
    DPRINT("**************Current position counter %f, before move relative **************",rpos1);
    
    // Deallochiamo tutto prima che termini il thread
    delete mySlit1;
    mySlit1 = NULL;
    delete errPtr;
    errPtr = NULL;
    
    return 0;
   
    //sleep(5);
    //pthread_exit((void*)errPtr);
}

void* function2(void* str){
    
    int* errPtr = new int; // Nessun errore..
    
    char* strInit =(char*)str;
    common::actuators::AbstractActuator* mySlit1 = new ActuatorTechnoSoft(); // ATTENZIONE: NON E' STATA GESTITA L'ECCEZIONE BAD_ALLOC
    
    int ret;
    if((ret=mySlit1->init((void*)strInit))!=0){
        DERR("*************Cannot init axis. In fact the value returned is ****************");
        * errPtr = -1;
    }
    // Lettura stato
    std::string desc;
    int status;
    if(mySlit1->getState(&status,desc)<0){
	DERR("**************Axis: error at first reading status**************");
        * errPtr = -2;
    }
   
    // Lettura posizione tramite encoder e counter
    double rpos=-1000000,rpos1=-1000000;
    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
        * errPtr = -3;
    } 
    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	DERR("************** Error at first position reading by encoder **************");
        * errPtr = -4;
    }
    DPRINT("**************Current position encoder %f, before move relative **************",rpos);
    DPRINT("**************Current position counter %f, before move relative **************",rpos1);
    
    DPRINT("************** Movement operation starting **************");
    if(mySlit1->moveRelativeMillimeters(10)<0){
	DERR("**************Error returned by movement operation **************");
        * errPtr = -5;
    }
    sleep(25);
    DPRINT("************** 1 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -6;
//    }
//    sleep(25);
//    DPRINT("************** 2 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -7;
//    }
//    sleep(25);
//    DPRINT("************** 3 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -8;
//    }
//    sleep(25); // Attesa completamento movimentazione, in seconds
//    DPRINT("************** 4 done **************");
//    if(mySlit1->moveRelativeMillimeters(10)<0){
//	DERR("**************Error returned by movement operation **************");
//        * errPtr = -9;
//    }
//    sleep(25);
//    DPRINT("************** 5 done **************");
//     if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//        * errPtr = -10;
//    } 
//    if(mySlit1->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//    	DERR("************** Error at first position reading by encoder **************");
//        * errPtr = -11;
//    }
    DPRINT("**************Current position encoder %f, before move relative **************",rpos);
    DPRINT("**************Current position counter %f, before move relative **************",rpos1);
    
    // Deallochiamo tutto prima che termini il thread
    delete mySlit1;
    mySlit1 = NULL;
    delete errPtr;
    errPtr = NULL;
    
    return 0;
   
    //sleep(5);
    //pthread_exit((void*)errPtr);
}


int main(int argc,const char* argv[]){
    
    int axis1;
    int axis2;
    
    double pos1,pos2;
    double rpos=-1000,rpos1=-1000;
    int ret;
    int status;
    
    const char *dev1,*conf1,*dev2,*conf2;
    char sinit1[256];
    char sinit2[256];
    if(argc!=5){
        USAGE;
        return -1;
    }
    // Inizializzazione parametri ASSE 1
    dev1=argv[1];        // [string], <dev/tty>
    conf1=argv[2];       // [string], <technosoft configuration>
    axis1=atoi(argv[3]); // [int], <axis>
    pos1=atof(argv[4]);  // [float], <move position in mm>
    PRINT("************ using axis %d, moving of %f mm**************",axis1,pos1);
    sprintf(sinit1,"%s,myslit1,%s,%d",dev1,conf1,axis1);
    common::actuators::AbstractActuator*mySlit1 = NULL;
    
    // Inizializzazione parametri ASSE 2
    dev2="/dev/ttyr1d";        // [string], <dev/tty>
    conf2=conf1;       // [string], <technosoft configuration>
    axis2=15; // [int], <axis>
    pos2=pos1;  // [float], <move position in mm>
    PRINT("************ using axis %d, moving of %f mm**************",axis2,pos2);
    sprintf(sinit2,"%s,myslit2,%s,%d",dev2,conf2,axis2);
    common::actuators::AbstractActuator*mySlit2 = NULL;
    
    pthread_t th1;
    pthread_t th2;
    
    pthread_create(&th1,NULL,function1,&sinit1[0]);
    //function1(&sinit1[0]);
    
    pthread_create(&th2,NULL,function2,&sinit2[0]);
    
    void* resp_th_1;
    void* resp_th_2;
    
    pthread_join(th1,NULL); //pthread_join modifica il contenuto del puntatore resp_th_1 
    pthread_join(th2,NULL);
    
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
////    int respHoming=1; // Operazione di homing non conclusa
////    int numHoming = 5;
////    
//    //sleep(10);
////    for(int i=1;i<=numHoming;i++){ // L'operazione di homing sara' eseguita piu volte consecutivamente, una volta che la precedente sia terminata indipendentemente
////        // con successo o insuccesso
////        DPRINT("*************Axis %d: Procedura di homing n. %d iniziata*************",axis1,i);
////        while(respHoming){ // Finche' la procedura di homing non e' completata con successo
////            DPRINT("********************Axis %d: Procedura di homing n. %d **********************",axis1,i);
////            respHoming = mySlit1->homing(common::actuators::AbstractActuator::homing2); // Il parametro in ingresso alla funzione non e' piu letto
////            usleep(1000);
////            if(respHoming<0){ 
////                DERR("***************Axis %d: Procedura di homing n. %d terminata con errore ***************",axis1,respHoming);   
////                break;
////            }
////        }
////        if(respHoming==0){
////            DPRINT("************Axis %d: Procedura di homing n. %d terminata con successo ***************",axis1,i);
////        }
////        respHoming = 1;
////        usleep(1000000);
////    }
//    
////    if(respHoming==0){
////        DPRINT("Ultima operazione di homing terminata con successo");
////    }
////    else{ 
////        DPRINT("Operazione di homing terminata con errore");
////    }
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