#include "iostream"
#include <common/debug/core/debug.h>
#include <common/actuators/core/AbstractActuator.h>
#include "ActuatorTechnoSoft.h"


using namespace common::actuators::models;
#define USAGE \
  printf("Usage is:%s <dev/tty> <axis> <move mosition in mm>\n",argv[0]);

int main(int argc,const char* argv[]){
    int axis;
    float pos;
    double rpos=0,rpos1=0;
    const char *dev;
    char sinit[256];
    if(argc!=4){
        USAGE;
        return -1;
    }
    dev=argv[1];
    axis=atoi(argv[2]);
    pos=atof(argv[3]);
    PRINT("* using axis %d, moving of %f mm",axis,pos);
    common::actuators::AbstractActuator*mySlit;
    
    
    
   
    mySlit = new ActuatorTechnoSoft();
    //mySlit[1] = new ActuatorTechnosoft();
    sprintf(sinit,"%s,myslit,/u2/dcs/prefs/MOV/setups/1setup001.t.zip,14",dev);
    if(mySlit->init((void*)sinit)!=0){
        DERR("## cannot init");
        delete mySlit;
    }
    
    
    mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,rpos);
    mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,rpos1);
    
    DPRINT("current position encoder %f, counter %f",rpos,rpos1);
    DPRINT("moving...");
    
    mySlit->moveRelativeMillimeters(pos);
    mySlit->getPosition(common::actuators::AbstractActuator::READ_ENCODER,rpos);
    mySlit->getPosition(common::actuators::AbstractActuator::READ_COUNTER,rpos1);
    
    DPRINT("current after position encoder %f, counter %f",rpos,rpos1);

    delete mySlit;
	
    return 0;
}
