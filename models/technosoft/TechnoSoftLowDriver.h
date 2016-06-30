#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <string>
#include <cmath>
#include <limits.h>
#include <string.h>


#if defined(WINDOWS) || defined(WIN32)
#	include <conio.h>
#	include "../../../include/TML_lib.h"
#	include <windows.h>
#	include "../../../include/tmlcomm.h"

#	if defined(__BORLANDC__)
#		pragma comment(lib, "../../../lib/TML_lib-borlandc.lib")
#	else
#		pragma comment(lib, "../../../lib/TML_lib.lib")
#	endif
#else
#	include <unistd.h>
#	include <errno.h>
#	include <fcntl.h>
#	include "TML_lib.h"
#	include "tmlcomm.h"
//int getch();
#endif

#if defined(WINDOWS) || defined(WIN32)
#define CHANNEL_NAME "COM1"
#else
#define CHANNEL_NAME "/dev/ttyr00" // == "COM1" in windows  // (da MDS)
#endif

#define CHANNEL_TYPE CHANNEL_RS232                          //(da MDS)
#define NAXIS 1
#undef HOST_ID
#define HOST_ID		 14                                  //(da MDS)
#define AXIS_ID_01	14                                  //(da MDS)
#define BAUDRATE	115200                              //(da MDS)

// Features of trapezoidal speed profile
#define SPEED_DEFAULT 400.0 // 30.0  [mm/s] 
#define ACCELERATION_DEFAULT 0.6 // 0.6 [mm/s^2]     
#define MAX_SPEED_DEFAULT 500.0    // [mm/s]              (da MDS) 
#define MAX_ACCELERATION_DEFAULT 2.0   // [mm/s]          (da MDS)

// Features of homing procedure
#define HIGH_SPEED_HOMING_DEFAULT 10.0 // [mm/s]
#define MAX_HIGHSPEED_HOMING_DEFAULT 15.0 // [mm/s]       (da MDS)
#define LOW_SPEED_HOMING_DEFAULT 1.0 // [mm/s]
#define MAXLOW_SPEED_HOMING_DEFAULT 3.0 // [mm/s]         (da MDS)
#define ACCELERATION_HOMING_DEFAULT 0.3 //[mm/s^2]
#define MAX_ACCELERATION_HOMING_DEFAULT 0.6 // [mm/s^2]   (da MDS)

//#define MAX_LENGTH_STRING_FROM_SHELL 50
//#define MAX_COMMAND_LENGTH 10
#define N_ENCODER_LINES_DEFAULT 800.0     // numero linee encoder                                     (da MDS)
#define CONST_MULT_TECHNOFT_DEFAULT 256.0 // numero micro steps per step                              (da MDS)
#define STEPS_PER_ROUNDS_DEFAULT 200.0     // numero steps per giro                                   (da MDS)
#define N_ROUNDS_DEFAULT 20.0              // numero giri per effettuare 1.5 mm (spostamento lineare) (da MDS)
#define LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT 1.5 //[mm] 

#include <map>
#include <boost/shared_ptr.hpp>

namespace common{
    
    namespace actuators{
       namespace models {
           
           class ElectricPowerException{
                public:
                    ElectricPowerException(){}
                    void badElectricPowerInfo();
           };
           
           class StopMotionException{
                public:
                    StopMotionException(){}
                    void badStopMotionInfo();
           };
           
           class OpeningChannelException{
                public:
                    OpeningChannelException(){}
                    void badOpeningChannelInfo();
           };
           
 
	    //Channel class
            class SerialCommChannelTechnosoft{
                
                private:
                    std::string pszDevName; // The communication channel to be opened
                    BYTE btType; // the type of the communication channel and the CAN-bus communication protocol
                    DWORD baudrate; // communication baud rate
                    int hostID;
                    int fd; // file descriptor of the channel
                public:
                    SerialCommChannelTechnosoft(){}
                    SerialCommChannelTechnosoft(int hostID, const std::string& pszDevName,BYTE btType=CHANNEL_TYPE,DWORD baudrate=BAUDRATE);
                    int init(int hostID, const std::string& pszDevName,const BYTE& btType,const DWORD& baudrate);
                    int getFD(); 
		    std::string  getDevName();
		    int getbtType();
		    int getbaudrate();
		    void PrintChannel();
                    ~SerialCommChannelTechnosoft();
                    int open();
                    void close();
            };
        
            //TechnoSoftLowDriver class
            class TechnoSoftLowDriver {

	    public:
                
                //int hostID;
                int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
                int axisRef;// handler
                
                std::string devName;
                
                double n_encoder_lines; 
                double const_mult_technsoft; 
                double steps_per_rounds;    
                double n_rounds;            
                double linear_movement_per_n_rounds;
                
                // Trapezoidal profile parameters for move relative and move absolute
                double speed_mm_s;
                double maxSpeed_mm_s; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                double acceleration_mm_s2;
                double maxAcceleration_mm_s2; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                BOOL isAdditive;
                short movement;
                short referenceBase;
                
                // Speed parameters regarding homing procedure
                double highSpeedHoming_mm_s; // The homing travel speed
                double lowSpeedHoming_mm_s;
                double maxHighSpeedHoming_mm_s; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                double maxLowSpeedHoming_mm_s;   // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                double accelerationHoming_mm_s2;
                double maxAccelerationHoming_mm_s2; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                BOOL isAdditiveHoming;
                short movementHoming;
                short referenceBaseHoming;
                 
                // Additional parameters for s-curve profile
                //long jerkTime;
                //short decelerationType;
                
                //char setupFilePath[200];
                
                // ************** cosa rappresentano queste tre variabili? ***************
                //int absoluteSteps;// contatore software
                //int status_register;// Reg_MER
                //int error_register; 
                
                //Encoder parameter
                //int encoderLines; // (passato da MDS)
                typedef boost::shared_ptr< SerialCommChannelTechnosoft > channel_psh;
                channel_psh my_channel;
                typedef std::map<std::string,channel_psh> channel_map_t;
                
                static channel_map_t channels; // gli oggetti TechnoSoftLowDriver condivideranno una struttura dati map,
                                              // percio e dichiarata di tipo static
                
                //bool alreadyopenedChannel;  // Canale di comunicazione aperto
                bool poweron; // alimentazione al drive motor erogata
                //bool channelJustOpened;
                
                // Transition parameters
                
            public:
                // Costruttore device channel and device name
                TechnoSoftLowDriver(int hostID, const std::string& devName, BYTE btType, DWORD baudrate);
                // *************ATTENZIONE, DICHIARARE IL METODO DISTRUTTORE******************** 
                ~TechnoSoftLowDriver();
                // Inizializzazione singolo drive/motor
                int init(const std::string& setupFilePath,
                        const int& axisID,
                        const double speed=SPEED_DEFAULT,
                        const double maxSpeed=MAX_SPEED_DEFAULT, 
                        const double acceleration=ACCELERATION_DEFAULT,
                        const double maxAcceleration = MAX_ACCELERATION_DEFAULT,
                        const BOOL isAdditive=FALSE, 
                        const short moveMoment =UPDATE_IMMEDIATE,
                        const short referenceBase=FROM_REFERENCE,
                        const double _highSpeedHoming=HIGH_SPEED_HOMING_DEFAULT,
                        const double _lowSpeedHoming=LOW_SPEED_HOMING_DEFAULT,
                        const double _maxHighSpeedHoming=MAX_HIGHSPEED_HOMING_DEFAULT,
                        const double _maxLowSpeedHoming=MAXLOW_SPEED_HOMING_DEFAULT,
                        const double _accelerationHoming=ACCELERATION_HOMING_DEFAULT,
                        const double _maxAccelerationHoming=MAX_ACCELERATION_HOMING_DEFAULT,
                        const BOOL _isAdditiveHoming=FALSE,
                        const short _movementHoming=UPDATE_IMMEDIATE,
                        const short _referenceBaseHoming=FROM_REFERENCE,
                        const double _n_encoder_lines=N_ENCODER_LINES_DEFAULT, 
                        const double _const_mult_technsoft=CONST_MULT_TECHNOFT_DEFAULT, 
                        const double _steps_per_rounds=STEPS_PER_ROUNDS_DEFAULT,    
                        const double _n_rounds=N_ROUNDS_DEFAULT,            
                        const double _linear_movement_per_n_rounds=LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT);

                int providePower();
                int stopPower();
                
                int moveRelativeSteps(const long& deltaPosition);// (0 -> OK)  (different 0 -> error)
                double getdeltaMicroSteps(const double& deltaMillimeters);
                //Set methods
                int moveRelativeStepsHoming(const long& deltaPosition);
                int moveVelocityHoming();
                
                // Set trapezoidal profile parameters
                int setSpeed(const double& speed);
                int setMaxSpeed(const double& maxspeed);
                int setAcceleration(const double& acceleration);
                int setMaxAcceleration(const double& maxAcceleration);
                int setAdditive(const BOOL& isAdditive);
                int setMovement(const short& movement);
                int setReferenceBase(const short& referenceBase);
                //Get methods
                int getSpeed(double& speed);
                  
                int getHighSpeedHoming(double& _highSpeedHoming);

                //Set homing parameters
                int sethighSpeedHoming(const double& _highSpeedHoming_mm_s);
                int setMaxhighSpeedHoming(const double& _speed);
                int setlowSpeedHoming(const double& _lowSpeedHoming_mm_s);
                int setMaxlowSpeedHoming(const double& speed);
                int setaccelerationHoming(const double& _accelerationHoming_mm_s2);
                int setMaxAccelerationHoming(const double& _maxaccelerationHoming_mm_s2);
                int setAdditiveHoming(const BOOL& isAdditive);
                int setMovementHoming(const short& movement);
                int setReferenceBaseHoming(const short& referenceBase);
                
                int setConst_mult_technsoft(double& _const_mult_technsoft);
                int setSteps_per_rounds(double& _steps_per_rounds);
                int setN_rounds(double& _n_rounds);
                int setLinear_movement_per_n_rounds(double& _linear_movement_per_n_rounds);
                int setEncoderLines(double& _encoderLines);
                
                //Encoder lines
                
                
                int moveAbsoluteSteps(const long& position) const;
                int moveAbsoluteStepsHoming(const long& position) const;
                
                // get methods for variables
                channel_psh getMyChannel();
                int getCounter(double* deltaPosition_mm);
                int getEncoder(double* deltaPosition_mm);
                // resetting methos
                int resetCounter();// reset TPOS_register();
                int resetEncoder();// reset APOS_register();
                int resetFault();
                int resetSetup();
                int getPower(BOOL& powered); //***************** Questo metodo dovrà essere sostituito da:
                //int getRegister()**********************;
                int stopMotion();
                int deinit();
                int setDecelerationParam(double deceleration);
                int setFixedVariable(LPCSTR pszName, double value);
                int abortNativeOperation();
                int executeTMLfunction(std::string& pszFunctionName);
                int setVariable(LPCSTR pszName, long value);
                int readHomingCallReg(short selIndex, WORD& status);
                int setEventOnLimitSwitch(short _lswType=LSW_NEGATIVE, short _transitionType=TRANSITION_HIGH_TO_LOW, BOOL _waitEvent=0, BOOL _enableStop=0);
                int setEventOnMotionComplete(BOOL waitEvent=0, BOOL enableStop=0);
                int setPosition(const long& posValue);
                
                int getLVariable(std::string& nameVar, long& var);
                
                int checkEvent(BOOL& event);
                
                int getStatusOrErrorReg(const short& regIndex, WORD& contentRegister, std::string& descrErr);

                int getMERregister();// read content of the error register
                int getSRLregister();// read content of the low part of the status register
                int getSRHregister();// read content of the high part of the status register
                //******************* da aggiungere la lettura dell'altro registro rimanente ******************
                
                int getFirmwareVers(char* firmwareVers);
                
                int selectAxis();
           };
	}// chiude namespace technosoft

    }
}

