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
    common::actuators::AbstractActuator*mySlit;
    
    mySlit = new ActuatorTechnoSoft(); 
    sprintf(sinit,"%s,myslit,%s,%d",dev,conf,axis);

    // Inizializzazione
    if((ret=mySlit->init((void*)sinit))!=0){
        DERR("*************Cannot init. In fact the value returned is %d ****************",ret);
        delete mySlit;
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
    DPRINT("************** Current position encoder %f, before movement **************",rpos);
    
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	fprintf(stderr,"**************Error at first position reading by counter **************\n");
        return -4;
    }
    DPRINT("************** Current position counter %f, before movement **************",rpos1);
    
    DPRINT("************** Prima movimentazione di 10 mm **************");
    // Spostamento della slitta 
    if(mySlit->moveRelativeMillimeters(10)<0){
	fprintf(stderr,"************** Error returned by movement operation **************\n");
        return -5;
    }
    	 
    sleep(30); // // Attesa completamento movimentazione, in seconds
    // Lettura posizione tramite encoder e counter
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
	fprintf(stderr,"**************Error at first position reading by encoder **************\n");
        return -3;
    }
    DPRINT("************** Current position encoder %f, before movement **************",rpos);
    
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	fprintf(stderr,"**************Error at first position reading by counter **************\n");
        return -4;
    }
    DPRINT("************** Current position counter %f, before movement **************",rpos1);
    
    uint64_t timeo_homing_ms = 20000;
       
    if(mySlit->setTimeoutHoming(timeo_homing_ms)<0){ //Settiamo il timeout = 100000
        return -6;
    }
      
    int respHoming;
    if((respHoming=mySlit->homing(common::actuators::AbstractActuator::defaultHoming))<0){
        fprintf(stderr,"************** Error returned by movement operation with code %d**************\n",respHoming);
        return -7;
    }
       
////    
////    int resp;
////    if((resp=mySlit->setTrapezoidalProfile(50, 0.2, 0, 1, 1))<0){
////        fprintf(stderr,"************** Error returned by setting Trapezoidal profile %d **************\n", resp);
////        return -7;
////    }
////    
////    if(mySlit->moveRelativeMillimeters(pos)<0){
////	fprintf(stderr,"************** Error returned by second movement operation **************\n");
////        return -8;
////    }
////    sleep(50); // // Attesa completamento movimentazione, in seconds
////    
////    printf("Movement absolute\n");
////    int resp2;
////    if((resp2=mySlit->moveAbsoluteMillimeters(15))<0){
////	fprintf(stderr,"************** Error returned by movement absolute operation with code %d **************\n", resp2);
////        return -9;
////    }
////    sleep(50); // Attesa completamento movimentazione, in seconds
////    
////    /*do{
////        mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos);
////        mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1);
////        printf("back ->%.5f\r",rpos1);
////    } while ((rpos1-.1) > 0 );
////    
////    printf("\n moving to %f...\n",pos);
////   */ 
////    /* mySlit->moveRelativeMillimeters(pos);
////    do{
////        mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos);
////        mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1);
////        printf("up ->%.5f\r",rpos1);
////    } while ((rpos1+.1) < po
////
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0){
	fprintf(stderr,"************** Error at second position after homing by encoder **************\n");
        return -10;
    }
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0){
    	fprintf(stderr,"************** Error at second position after homing reading by counter **************\n");
        return -11;
    }
    DPRINT("************** current position encoder: %f, and counter %f after movement **************",rpos,rpos1);
    
    //mySlit->stopMotion();
    delete mySlit;
	
    //sleep(5);
    return 0;
}