#include "iostream"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "ActuatorTechnoSoft.h"


using namespace common::actuators::models;
#define USAGE \
  printf("Usage is:%s <dev/tty> <technosoft configuration> <axis> <move mosition in mm>\n",argv[0]);

int main(int argc,const char* argv[]){
    int axis;
    float pos;
    double rpos=0,rpos1=0;
    int ret;
    const char *dev,*conf;
    char sinit[256];
    if(argc!=5){
        USAGE;
        return -1;
    }
    dev=argv[1];
    conf=argv[2];
    axis=atoi(argv[3]);
    pos=atof(argv[4]);
    PRINT("* using axis %d, moving of %f mm",axis,pos);
    common::actuators::AbstractActuator*mySlit;
    
    
    
   
    mySlit = new ActuatorTechnoSoft();
    //mySlit[1] = new ActuatorTechnosoft();
    sprintf(sinit,"%s,myslit,%s,%d",dev,conf,axis);
    if((ret=mySlit->init((void*)sinit))!=0){
        DERR("cannot init ret=%d",ret);
        delete mySlit;
        return -1;
    }
    
    
    mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,rpos);
    mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,rpos1);
    DPRINT("current position encoder %f, counter %f, moving back...",rpos,rpos1);
    mySlit->moveRelativeMillimeters(-rpos1);
    do{
        mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,rpos);
        mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,rpos1);
        printf("back ->%f",rpos1);
    } while ((rpos1-.1) < 0 );
    
    DPRINT("moving...");
    
    mySlit->moveRelativeMillimeters(pos);
    do{
        mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,rpos);
        mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,rpos1);
        printf("->%f",rpos1);
    } while ((rpos1+.1) < pos);
    
    DPRINT("current after position encoder %f, counter %f",rpos,rpos1);
    delete mySlit;
	
    return 0;
}
