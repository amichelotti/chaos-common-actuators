
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
    int setHost(bool);// TRUE se Host
  int initComunication();// puo’ instaurarla solo l’host
  int moveRelativeSteps(int, int, int);// (0 -> OK)  (≠0 -> error)
  int setVelocity(int);
  int setAcceleration(int);
  int setProfile(int);// Moto Trapezoidale o SCurve
  int stop();
  int resetCounter();// TPOS_register();
  int resetEncoder();// APOS_register();
  int read_register();// REG_MER_register();
  
  // int readEndRun_in();
  // int readEndRun_out();
  // int readEmergency();
  // int readI2t();
  // int read overCurrent();
 
 private:
  bool host;// TRUE per l’Host; solo l’Host puo’ inizializzare la comunicazione seriale
  int ID_axis;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
  int axis_ref;// handler
  char setup_file[100];
  int absoluteSteps;// contatore software
  int Velocity;
  int Acceleration;
  int status_register;// Reg_MER
  int error_register; 
  int fd;
};
