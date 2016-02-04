#include "iostream"
#include <common/actuators/core/AbstractSlit.h>
#include "TechnoSoftLowDriver.h"

int main(int argc,const char* argv[]){
	
    common::actuators::core::AbstractSlit*mySlit[3];
    
    common::actuators::core::ActuatorTechnosoft::technoinfo_t info;
    
    
   
    mySlit[0] = new ActuatorTechnosoft("/dev/tty","X");
    mySlit[1] = new ActuatorTechnosoft("/dev/tty","Y");
    
    mySlit[0]->init();
    mySlit[1]->init();

    
    while(1){
        mySlit[0]->move
        mySlit[0]->move
    }
    mySlit->init((void*)&info);
    
	
    return 0;
}
