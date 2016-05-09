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
//int getch();
#endif

#if defined(WINDOWS) || defined(WIN32)
#define CHANNEL_NAME "COM1"
#else
#define CHANNEL_NAME "/dev/ttyr00" // == "COM1" in windows
#endif

#define CHANNEL_TYPE CHANNEL_RS232
#define NAXIS 1
#undef HOST_ID
#define HOST_ID		0
#define AXIS_ID_01		14
#define BAUDRATE	115200

// Features of trapezoidal speed profile
#define SPEED_DEFAULT 400.0 // 30.0  [mm/s] 
#define ACCELERATION_DEFAULT 0.1 // 0.6 [mm/s^2]     
#define MAX_SPEED_DEFAULT 500.0    // [mm/s]               
#define MAX_ACCELERATION_DEFAULT 2.0   // [mm/s]          

// Features of homing procedure
#define HIGH_SPEED_HOMING_DEFAULT 10.0 // [mm/s]
#define MAX_HIGHSPEED_HOMING_DEFAULT 15.0 // [mm/s]
#define LOW_SPEED_HOMING_DEFAULT 1.0 // [mm/s]
#define MAXLOW_SPEED_HOMING_DEFAULT 3.0 // [mm/s]
#define ACCELERATION_HOMING_DEFAULT 0.3 //[mm/s^2]
#define MAX_ACCELERATION_HOMING_DEFAULT 0.6 // [mm/s^2]

//#define MAX_LENGTH_STRING_FROM_SHELL 50
//#define MAX_COMMAND_LENGTH 10
#define N_ENCODER_LINES_DEFAULT 800.0 //[mm]

#define CONST_MULT_TECHNOFT_DEFAULT 256.0 // numero micro steps per step (MDS)
#define STEPS_PER_ROUNDS_DEFAULT 200.0     // numero steps per giro
#define N_ROUNDS_DEFAULT 20.0 
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

	    //Channel class
            class SerialCommChannelTechnosoft{
                
                private:
                    std::string pszDevName; // The communication channel to be opened
                    BYTE btType; // the type of the communication channel and the CAN-bus communication protocol
                    DWORD baudrate; // communication baud rate
                    int fd; // file descriptor of the channel
                public:
                    SerialCommChannelTechnosoft(){}
                    SerialCommChannelTechnosoft(const std::string& pszDevName,const BYTE btType=CHANNEL_RS232,const DWORD baudrate=BAUDRATE);
                    int init(const std::string& pszDevName,const BYTE& btType,const DWORD& baudrate);
                    int getFD(); 
		    std::string  getDevName();
		    int getbtType();
		    int getbaudrate();
		    void PrintChannel();
                    ~SerialCommChannelTechnosoft();
                    int open(int hostID=HOST_ID);
                    void close();
            };
        
            //TechnoSoftLowDriver class
            class TechnoSoftLowDriver {

	    private:
                int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
                int axisRef;// handler
                
                std::string dev;
                std::string devName;
                
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
                short decelerationType;
                
                char setupFilePath[200];
                
                // ************** cosa rappresentano queste tre variabili? ***************
                int absoluteSteps;// contatore software
                int status_register;// Reg_MER
                int error_register; 
                
                //Encoder parameter
                int encoderLines; // (passato da MDS)
                typedef boost::shared_ptr< SerialCommChannelTechnosoft > channel_psh;
                channel_psh my_channel;
                typedef std::map<std::string,channel_psh> channel_map_t;
                
                static channel_map_t channels; // gli oggetti TechnoSoftLowDriver condivideranno una struttura dati map,
                                              // percio e dichiarata di tipo static
                
                bool alreadyopenedChannel; 
                bool poweron;
                
                // Transition parameters
                
            public:
                // Costruttore device channel and device name
                TechnoSoftLowDriver(const std::string dev, const std::string devName);
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
                        const double encoderLines=N_ENCODER_LINES_DEFAULT);
                
                //LONG RelPosition, DOUBLE Speed, DOUBLE Acceleration, BOOL IsAdditive, SHORT MoveMoment, SHORT ReferenceBase)
                //void setupTrapezoidalProfile(long, double, double, BOOL, short, short);
                int providePower();
                int stopPower();
                
                int moveRelativeSteps(const long& deltaPosition);// (0 -> OK)  (different 0 -> error)
                //Set methods
                int moveRelativeStepsHoming(const long& deltaPosition);
                int moveVelocityHoming();
                
                // Set trapezoidal profile parameters
                int setSpeed(const double& speed);
                int setAcceleration(const double& acceleration);
                int setAdditive(const BOOL& isAdditive);
                int setMovement(const short& movement);
                int setReferenceBase(const short& referenceBase);
                //Get methods
                int getSpeed(double& speed);
                  
                int getHighSpeedHoming(double& _highSpeedHoming);
                
                
                //Set homing parameters
                int sethighSpeedHoming(const double& _highSpeedHoming_mm_s);
                int setlowSpeedHoming(const double& _lowSpeedHoming_mm_s);
                int setaccelerationHoming(const double& _accelerationHoming_mm_s2);
                int setAdditiveHoming(const BOOL& isAdditive);
                int setMovementHoming(const short& movement);
                int setReferenceBaseHoming(const short& referenceBase);
                
                
                //Encoder lines
                int setEncoderLines(int& _encoderLines);
                
                int moveAbsoluteSteps(const long& position) const;
                int moveAbsoluteStepsHoming(const long& position) const;
                
                // get methods for variables
                channel_psh getMyChannel();
                int getCounter(long& tposition);
                int getEncoder(long& aposition);
                // resetting methos
                int resetCounter();// reset TPOS_register();
                int resetEncoder();// reset APOS_register();
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
           };
	}// chiude namespace technosoft

    }
}

