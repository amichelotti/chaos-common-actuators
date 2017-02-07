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
#define SPEED_DEFAULT 0.3      //     [mm/s]     (50.0 [IU]) 
#define ACCELERATION_DEFAULT 1.373 //    [mm/s2]    ( 0.6  [IU])     
#define MAX_SPEED_DEFAULT 0.824  //      [mm/s]    (450.0 [IU])             
#define MAX_ACCELERATION_DEFAULT 4.577 //[mm/s2]     (2.0 [IU])     
// Features of homing procedure
#define HIGH_SPEED_HOMING_DEFAULT 0.2  // [mm/s]    (10.0 [IU])          
#define MAX_HIGHSPEED_HOMING_DEFAULT 0.5 // [mm/s]  (15.0 [IU])
#define LOW_SPEED_HOMING_DEFAULT 0.002   // [mm/s]     (1.0 [IU])
#define MAXLOW_SPEED_HOMING_DEFAULT 0.006 //[mm/s]     (3.0 [IU])
#define ACCELERATION_HOMING_DEFAULT 0.687 //[mm/s2]    (0.3 [IU])
#define MAX_ACCELERATION_HOMING_DEFAULT 1.372 // [mm/s2]    (0.6 [IU])

#define N_ENCODER_LINES_DEFAULT 800.0     // numero linee encoder                                     (da MDS)
#define CONST_MULT_TECHNOFT_DEFAULT 256.0 // numero micro steps per step                              (da MDS)
#define STEPS_PER_ROUNDS_DEFAULT 200.0     // numero steps per giro                                   (da MDS)
#define N_ROUNDS_DEFAULT 20.0              // numero giri per effettuare 1.5 mm (spostamento lineare) (da MDS)
#define LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT 1.5 //[mm] 
#define CONVERSION_FACTOR_DEG_UI 8.789 //[mm]
#define CONVERSION_FACTOR_DEGs2_UI 10986 //[mm]
//#define REDUCTION_FACTOR 13.3333333333
#define FULLSCALE_POTENTIOMETER 20.0
#define V_LNS 7.7 //[V]
#define V_LSP 0.3 //[V]
#define RANGE 1 //[m]
#define HARD_RESET_MODE_DEFAULT false //[m]

#include <map>
//#include <boost/shared_ptr.hpp>

namespace common{
    
    namespace actuators{
       namespace models {
           
//           class ElectricPowerException{
//                public:
//                    ElectricPowerException(){}
//                    void badElectricPowerInfo();
//           };
//           
//           class StopMotionException{
//                public:
//                    StopMotionException(){}
//                    void badStopMotionInfo();
//           };
//           
//           class OpeningChannelException{
//                public:
//                    OpeningChannelException(){}
//                    void badOpeningChannelInfo();
//           };
           
	    //Channel class
            class SerialCommChannelTechnosoft{
                
                public:                    
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

	    private:
                
                int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
                int axisRef;// handler
                
                std::string devName;
                
                bool controlLNS;
                
                double n_encoder_lines; 
                double const_mult_technsoft; 
                double steps_per_rounds;    
                double n_rounds;            
                double linear_movement_per_n_rounds;
                
                // Trapezoidal profile parameters for move relative and move absolute
                //double speed_mm_s;
                double speed_IU; //    [IU]  
                double maxSpeed_IU; // [IU] 
                double acceleration_IU;
                double maxAcceleration_IU; 
                BOOL isAdditive;
                short movement;
                short referenceBase;
                
                // Speed parameters regarding homing procedure
                double highSpeedHoming_IU; // The homing travel speed
                double lowSpeedHoming_IU;
                double maxHighSpeedHoming_IU; 
                double maxLowSpeedHoming_IU;   
                double accelerationHoming_IU;
                double maxAccelerationHoming_IU; 
                BOOL isAdditiveHoming;
                short movementHoming;
                short referenceBaseHoming;
                
                double voltage_LNS; //[V]
                double voltage_LPS; //[V]
                double range;
                double fullScalePot;
                double constantPot;
                
                struct timeval lastTimeTakenForHoming;
                double minimumIntervalForHoming; // in seconds
                
                bool hardResetMode;
                
                
                
                //bool controlledInitialPositionHoming; 
                //double epsylon;
//                int stateHoming0;
//                int stateHoming1;
//                int stateHoming2;
//                int stateHoming3;
//                int stateHoming4;
//                int stateHoming5;
                long cap_position;
                
            public:
                int internalHomingStateDefault; // N.B. Per ragioni di efficienza questo membro e' utile che rimanga pubblico
                int internalHomingStateHoming2; // N.B. Per ragioni di efficienza questo membro e' utile che rimanga pubblico
                
                bool readyState;
                 
                // Transition parameters
                
            public:
                // Costruttore device channel and device name
                TechnoSoftLowDriver();
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
                        const double _linear_movement_per_n_rounds=LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT,
                        const double voltage_LNS = V_LNS, //[V]
                        const double voltage_LPS = V_LSP, //[V]
                        const double range = RANGE,  //[meter]
                        const double fullScalePot = FULLSCALE_POTENTIOMETER //[V] 
                        );
                
                //double speedfromIUTOMMs(double _speed_IU);
                //double accelerationfromIUToMMs(double _acceleration_IU);
                
                
                int homing(int mode);
                
                int getinternalHomingStateDefault();
                int getinternalHomingStateHoming2();
                
                int getEmergency(BYTE nIO, BYTE& inValue, std::string& descrErr);

                int providePower();
                int stopPower();
                
                int moveRelativeSteps(const long& deltaPosition);// (0 -> OK)  (different 0 -> error)
                double getdeltaMicroSteps(const double& deltaMillimeters);
                //Set methods
                int moveRelativeStepsHoming(const long& deltaPosition);
                int moveVelocityHoming();
                
                // Set trapezoidal profile parameters
                int setSpeed(const double& speed);
                int getSpeed(double& speed);
                
                int setMaxSpeed(const double& maxspeed);
                int getMaxSpeed(double& maxspeed);
                
                int setAcceleration(const double& acceleration);
                int getAcceleration(double& acceleration);
                
                int setMaxAcceleration(const double& maxAcceleration);
                int getMaxAcceleration(double& maxAcceleration);

                int setAdditive(const BOOL& isAdditive);
                int getAdditive(BOOL& isAdditive);

                int setMovement(const short& movement);
                int getMovement(short& movement);
                
                int setReferenceBase(const short& referenceBase);
                int getReferenceBase(short& referenceBase);
                
                //Set homing parameters
                int sethighSpeedHoming(const double& _highSpeedHoming_mm_s);
                int getHighSpeedHoming(double& _highSpeedHoming);
                
                int setMaxhighSpeedHoming(const double& _speed);
                int getMaxhighSpeedHoming(double& _speed);
                
                int setlowSpeedHoming(const double& _lowSpeedHoming_mm_s);
                int getlowSpeedHoming(double& _lowSpeedHoming_mm_s);
                
                int setMaxlowSpeedHoming(const double& speed);
                int getMaxlowSpeedHoming(double& _lowSpeedHoming_mm_s);
                
                int setaccelerationHoming(const double& _accelerationHoming_mm_s2);
                int getaccelerationHoming(double& _accelerationHoming_mm_s2);
                
                int setMaxAccelerationHoming(const double& _maxaccelerationHoming_mm_s2);
                int getMaxAccelerationHoming(double& _maxaccelerationHoming_mm_s2);
                           
                int setConst_mult_technsoft(const double& _const_mult_technsoft);
                int getConst_mult_technsoft(double& _const_mult_technsoft);
                
                int setSteps_per_rounds(const double& _steps_per_rounds);
                int getSteps_per_rounds(double& _steps_per_rounds);
                
                int setN_rounds(const double& _n_rounds);
                int getN_rounds(double& _n_rounds);
                
                int setLinear_movement_per_n_rounds(const double& _linear_movement_per_n_rounds);
                int getLinear_movement_per_n_rounds(double& _linear_movement_per_n_rounds);
                
                int setEncoderLines(const double& _encoderLines);
                int getEncoderLines(double& _encoderLines);
                
                int setFullscalePot(const double& _fullScale);
                int getFullscalePot(double& _fullScale);
                
                //Encoder lines

                int moveAbsoluteSteps(const long& position);
                int moveAbsoluteStepsHoming(const long& position) const;
                int setvoltage_LNS(const double& _voltage_LNS);
                int getvoltage_LNS(double& _voltage_LNS);
                
                int setvoltage_LPS(const double& _voltage_LPS);
                int getvoltage_LPS(double& _voltage_LNS);
                
                int setRange(const double& _range);
                int getRange(double& _range);
                // get methods for variables
                //channel_psh getMyChannel();
                int getCounter(double* deltaPosition_mm);
                int getEncoder(double* deltaPosition_mm);
                int getPotentiometer(double* voltage);
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
                
                int hardreset(bool mode=HARD_RESET_MODE_DEFAULT);
                
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

