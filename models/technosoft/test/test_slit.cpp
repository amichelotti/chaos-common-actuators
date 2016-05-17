#include "iostream"
#include "stdio.h"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "ActuatorTechnoSoft.h"


using namespace common::actuators::models;
#define USAGE \
  printf("**************Usage is:%s <dev/tty> <technosoft configuration> <axis> <move position in mm>*************\n",argv[0]);

int main(int argc,const char* argv[]){
    
    int axis;
    double pos;
    double rpos=-1000,rpos1=-1000;
    int ret;
    int status;
    
    const char *dev,*conf;
    char sinit[256];
    if(argc!=5){
        USAGE;
        return -1;
    }
    dev=argv[1];        // [string], <dev/tty>
    conf=argv[2];       // [string], <technosoft configuration>
    axis=atoi(argv[3]); // [int], <axis>
    pos=atof(argv[4]);  // [float], <move position in mm>
    PRINT("************ using axis %d, moving of %f mm**************",axis,pos);
    common::actuators::AbstractActuator*mySlit = NULL;
    
    mySlit = new ActuatorTechnoSoft(); // ATTENZIONE: NON E' STATA GESTITA L'ECCEZIONE BAD_ALLOC
    sprintf(sinit,"%s,myslit,%s,%d",dev,conf,axis);

    // Inizializzazione
    if((ret=mySlit->init((void*)sinit))!=0){
        DERR("*************Cannot init. In fact the value returned is %d ****************",ret);
        return -1;
    }
    DPRINT("************Operazione di inizializzazione andata a buon fine!***************");
   
    // Lettura stato
    std::string desc;
    if(mySlit->getState(&status,desc)<0){
	fprintf(stderr,"**************Error at first reading status**************\n");
        return -2;
    }
    fprintf(stderr,"**************First reading status %d, %s **************\n",status ,desc.c_str());
  
    // Lettura posizione tramite encoder e counter
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
	fprintf(stderr,"**************Error at first position reading by encoder **************\n");
        return -3;
    }
    //DPRINT("************** Current position encoder %f, before movement **************",rpos);
    
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	fprintf(stderr,"**************Error at first position reading by counter **************\n");
        return -4;
    }
    DPRINT("************** Current position encoder %f, before move relative **************",rpos);
    DPRINT("************** Current position counter %f, before move relative **************",rpos1);
    
    DPRINT("************** Reset alarms before move relative **************");
    if(mySlit->resetAlarms(0)<0){
    	fprintf(stderr,"************** Error setting alarms **************\n");
        return -5;
    }
    
    if(mySlit->getState(&status,desc)<0){
	fprintf(stderr,"**************Error at first reading status**************\n");
        return -2;
    }
    fprintf(stderr,"**************reading status after setting alarms %d, %s **************\n",status ,desc.c_str());
    
//    if(mySlit->getState(&status,desc)<0){
//	fprintf(stderr,"**************Error get status after reset fault**************\n");
//        return -6;
//    }
//    DPRINT("************** Reading status after reset alarms: %d, %s **************\n",status ,desc.c_str());
    
    
    DPRINT("************** Prima movimentazione di 10 mm **************");
    // Spostamento della slitta 
    if(mySlit->moveRelativeMillimeters(10)<0){
	fprintf(stderr,"************** Error returned by movement operation **************\n");
        return -7;
    }
    	 
    sleep(30); // Attesa completamento movimentazione, in seconds
    
    DPRINT("**************Move relative finished**************\n");
    
    if(mySlit->getState(&status,desc)<0){
	fprintf(stderr,"**************Error reading status after move relative**************\n");
        return -8;
    }
    fprintf(stderr,"**************Reading status %d, %s after move relative**************\n",status ,desc.c_str());
    
    // Lettura posizione tramite encoder e counter
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
	fprintf(stderr,"**************Error position reading by encoder after move relative **************\n");
        return -9;
    }
    DPRINT("************** Current position encoder %f, after move relative  **************",rpos);
    
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	fprintf(stderr,"**************Error position reading by counter, after move relative **************\n");
        return -10;
    }
    DPRINT("************** Current position encoder %f, after move relative **************",rpos);
    DPRINT("************** Current position counter %f, after move relative **************",rpos1);
    
    
////    uint64_t timeo_homing_ms = 20000;
////       
////    if(mySlit->setTimeoutHoming(timeo_homing_ms)<0){ //Settiamo il timeout = 100000
////        return -6;
////    }
//      
//    int respHoming;
//    if((respHoming=mySlit->homing(common::actuators::AbstractActuator::homing2))<0){
//        fprintf(stderr,"************** Error returned by movement operation with code %d**************\n",respHoming);
//        return -7;
//    }
//      
//    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
//	fprintf(stderr,"************** Error at second position after homing by encoder **************\n");
//        return -10;
//    }
//    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
//    	fprintf(stderr,"************** Error at second position after homing reading by counter **************\n");
//        return -11;
//    }
//    DPRINT("************** Current position encoder %f, after homing**************",rpos);
//    DPRINT("************** Current position counter %f, after homing **************",rpos1);
//    
//    if(mySlit->getState(&status,desc)<0){
//	fprintf(stderr,"**************Error at reading status after homing**************\n");
//        return -14;
//    }
//    fprintf(stderr,"**************Reading status %d, %s after homing **************\n",status ,desc.c_str());
    
    try {
        if(mySlit!=NULL){
            delete mySlit;
            DPRINT("Effect of the complete deallocation: possible motion stopped (if channel has been opened); possible electric power interrupted (if channel has been opened); possible opened communication channel closed.");
        }
    }
    catch(StopMotionException e){
        e.badStopMotionInfo();
        return -11;
    }
    catch(ElectricPowerException e){
        e.badElectricPowerInfo();
        return -12;
    }
    
    sleep(600); //Sleep inserito per analizzare la corretta deallocazione delle risorse
	
    //sleep(5);
    return 0;
}