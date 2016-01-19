#include "TML_lib.h"
#include "TechnoSoftLowDriver.h"
using namespace common::actuators::technosoft;
int TechnoSoftLowDriver::init(char*path){

  if(!initCommunicationChannel(&fd))
    {
      fprintf(stderr, " Communication error! \n %s\n", TS_GetLastErrorText());
      return -1;
    }
  idAxis  = TS_LoadSetup(path);
  if(idAxis<0){
    return -2;
  }
  if(!TS_SetupAxis(AXIS_ID_01, idAxis)) 
    return -3;
  
  return 0;
}
   

  

int TechnoSoftLowDriver::initializeAxis(bool host,int axisid){
}
 int TechnoSoftLowDriver::setHost(bool ret){
     return 0;
             
 }
  int TechnoSoftLowDriver::initComunication(){
      return 0;
  }
  int  TechnoSoftLowDriver::moveRelativeSteps(int a , int b, int c){
      return 0;
  }
  int TechnoSoftLowDriver::setVelocity(int v){return 0;}
  int TechnoSoftLowDriver::setAcceleration(int a ){return 0;}
  int TechnoSoftLowDriver::setProfile(int p){return 0;}
  int TechnoSoftLowDriver::stop(){ return 0;}
  int TechnoSoftLowDriver::resetCounter(){return 0;}
  int TechnoSoftLowDriver::resetEncoder(){ return 0;}
  int TechnoSoftLowDriver::read_register(){return 0;}
  