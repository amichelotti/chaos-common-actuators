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
    float pos;
    float rpos=-1000,rpos1=-1000;
    int ret;
    int status;
    std::string desc;
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
    else{
	DPRINT("************Operazione di inizializzazione andata a buon fine!***************");
    }

    // Lettura stato
    if(mySlit->getState(&status,desc)<0)
	fprintf(stderr,"**************Error at first reading status**************\n");
    else
    	fprintf(stderr,"**************First reading status %d, %s **************\n",status ,desc.c_str());
    
    // Lettura posizione tramite encoder e counter
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0)
	fprintf(stderr,"**************Error at first position reading by encoder **************\n");
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0)
    	fprintf(stderr,"**************Error at first position reading by counter **************\n");
    
    DPRINT("************** Current position encoder %f, counter %f, before movement **************",rpos,rpos1);

    // Spostamento della slitta 
    if(mySlit->moveRelativeMillimeters(pos)<0)
	fprintf(stderr,"************** Error returned by movement operation **************\n");
	 
    sleep(30); // // Attesa completamento movimentazione, in seconds

    /*do{
        mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos);
        mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1);
        printf("back ->%.5f\r",rpos1);
    } while ((rpos1-.1) > 0 );
    
    printf("\n moving to %f...\n",pos);
   */ 
    /* mySlit->moveRelativeMillimeters(pos);
    do{
        mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos);
        mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1);
        printf("up ->%.5f\r",rpos1);
    } while ((rpos1+.1) < pos);
    */ 

    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,&rpos)<0)
	fprintf(stderr,"************** Error at second position reading by encoder **************\n");
    if(mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,&rpos1)<0)
    	fprintf(stderr,"************** Error at second position reading by counter **************\n");
    DPRINT("************** current position encoder: %f, and counter %f after movement **************",rpos,rpos1);
    
    // test procedura di homing
    //AbstractActuator::homingType homeType = AbstractActuator::nativeHoming15; 
    //mySlit->homing(homeType);
    
    delete mySlit;
	
    return 0;
}
