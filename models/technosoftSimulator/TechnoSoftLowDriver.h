/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TechnoSoftLowDriver.h
 * Author: caschera
 *
 * Created on August 31, 2016, 2:48 PM
 */

#ifndef TECHNOSOFTLOWDRIVER_H
#define TECHNOSOFTLOWDRIVER_H

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
#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC
#endif
#include <pthread.h>
#include <math.h>       /* fabs */
#include <boost/random.hpp>
//#include <boost/random/normal_distribution.hpp>
//#include <boost/random/mersenne_twister.hpp>
//#include <boost/random/variate_generator.
//#include <boost/math/distributions/normal.hpp>


#if defined(WINDOWS) || defined(WIN32)
#	include <conio.h>
#	include "TML_lib.h"
#	include <windows.h>
//#	include "tmlcomm.h"

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
//#define SPEED_DEFAULT 400.0 // 30.0  [mm/s]
#define SPEED_DEFAULT 400 //  [microstep/ms]
#define PERCNOISE_DEFAULT 0.01

#define ACCELERATION_DEFAULT 0.6 // 0.6 [mm/s^2]
#define MAX_SPEED_DEFAULT 500.0    // [mm/s]              (da MDS)
#define MAX_ACCELERATION_DEFAULT 2.0   // [mm/s]          (da MDS)

// Features of homing procedure
//#define HIGH_SPEED_HOMING_DEFAULT 10.0 // [mm/s]
#define HIGH_SPEED_HOMING_DEFAULT 425.0 // [microstep/ms]

#define MAX_HIGHSPEED_HOMING_DEFAULT 600.0 // [mm/s]       (da MDS)
//#define LOW_SPEED_HOMING_DEFAULT 1.0 // [mm/s]
#define LOW_SPEED_HOMING_DEFAULT 50.0 // [microstep/ms]
#define MAXLOW_SPEED_HOMING_DEFAULT 150.0 // [mm/s]         (da MDS)
#define ACCELERATION_HOMING_DEFAULT 0.3 //[mm/s^2]
#define MAX_ACCELERATION_HOMING_DEFAULT 0.6 // [mm/s^2]   (da MDS)

#define N_ENCODER_LINES_DEFAULT 800.0      // numero linee encoder                                     (da MDS)
#define CONST_MULT_TECHNOFT_DEFAULT 256.0  // numero micro steps per step                              (da MDS)
#define STEPS_PER_ROUNDS_DEFAULT 200.0     // numero steps per giro                                   (da MDS)
#define N_ROUNDS_DEFAULT 20.0              // numero giri per effettuare 1.5 mm (spostamento lineare) (da MDS)
#define LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT 1.5 //[mm]

#define CONVERSION_FACTOR_DEG_UI 8.789 //[mm]
#define CONVERSION_FACTOR_DEGs2_UI 10986 //[mm]

#define POSITIVE_LIMIT_POSITION_DEFAULT 60000000
#define PERC_NOISE_DEFAULT 0.0
#define DURATION_ALARMS_INTERVAL_DEFAULT 60.0
#define ALARMS_PRESENT_DEFAULT false  


#define FULLSCALE_POTENTIOMETER 20.0
#define V_LNS 7.7 //[V]
#define V_LSP 0.3 //[V]
#define RANGE 500 //[mm]

#define HARD_RESET_MODE_DEFAULT false
#define PROBABILITY_OPERATION_ERROR 0.0

#include <map>
#include <boost/shared_ptr.hpp>

namespace common{

    namespace actuators{
       namespace models {
           namespace simul {

           struct containerIncrementPosition{
               long deltaPosition;
               //long absolutePosition;
           //    pthread_mutex_t mu;
           };

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

                typedef boost::uniform_real<> NumberDistribution;
                typedef boost::mt19937 RandomNumberGenerator;
                typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

                RandomNumberGenerator generator;

                int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
                int axisRef;// handler

                std::string devName;

                double n_encoder_lines;
                double const_mult_technsoft;
                double steps_per_rounds;
                double n_rounds;
                double linear_movement_per_n_rounds;

                // Trapezoidal profile parameters for move relative and move absolute
//                double speed_ms_s;
//                double maxSpeed_mm_s; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
//                double acceleration_mm_s2;
//                double maxAcceleration_mm_s2; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                
                double speed_IU; //    [IU]  
                double maxSpeed_IU; // [IU] 
                double acceleration_IU;
                double maxAcceleration_IU;
                BOOL isAdditive;
                short movement;
                short referenceBase;
		bool useUI;

                // Speed parameters regarding homing procedure
//                double highSpeedHoming_mm_s; // The homing travel speed
//                double lowSpeedHoming_mm_s;
//                double maxHighSpeedHoming_mm_s; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
//                double maxLowSpeedHoming_mm_s;   // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
//                double accelerationHoming_mm_s2;
//                double maxAccelerationHoming_mm_s2; // VALORE CHE UNA VOLA INIZIALIZZATO, NON PUO' ESSERE PIU CAMBIATO
                
                double highSpeedHoming_IU; 
                double lowSpeedHoming_IU;
                double maxHighSpeedHoming_IU; 
                double maxLowSpeedHoming_IU;   
                double accelerationHoming_IU;
                double maxAccelerationHoming_IU; 
                BOOL isAdditiveHoming;
                short movementHoming;
                short referenceBaseHoming;

                 
                bool stopMotionCommand;
                long position; // expressed in microsteps
                long positionCounter;
                long positionEncoder;
                long positionPotentiometer;
                long cap_position;
                bool LNStransition;
                bool LPStransition;
                
                bool controlLNS;

                long epsylon;
                double p;
                
                bool deallocateTimerAlarms;
                bool deallocateTimerStates;

                // Stato esecuzione thread
//                bool threadMoveRelativeOn;
//                bool threadMoveAbsoluteOn;

                int motionscalled;

                //std::default_random_engine generator;

                //containerIncrementPosition cIP;
                long absolutePosition;
                long deltaPosition;
             //   pthread_mutex_t mu;
                
                pthread_t thstaticFaultsGeneration,th;

                int moveAbsolutePosition();
                int incrDecrPosition();
                int moveConstantVelocityHoming();

                static void* staticIncrDecrPositionFunctionForThread(void*);
                //static void* staticIncrDecrPositionHomingFunctionForThread(void*);
                static void* staticMoveConstantVelocityHomingFunctionForThread(void*);
                static void* staticMoveAbsolutePositionHomingFunctionForThread(void*);
                static void* staticMoveAbsolutePositionForThread(void*);
                int moveAbsolutePositionHoming();
                
                int faultsGeneration();
                bool alarms;
                
                static void* staticFaultsGeneration(void* objPointer);
                
                int resetStatesTimer();
                void* staticResetStatesTimerForThread(void* objPointer);
                
                WORD contentRegisterMER;
                
                //int homingType;

                //long deltaNoise;
                double percNoise;
                bool homingStopped;
                
                bool controlledPositionHoming;
                long positiveLimitPosition;
                
                double durationAlarmsInterval;

            public:
                bool alarmsInfoRequest;
                bool regMERrequest;
                WORD contentRegMER; // Fault driver register
                bool regSRHrequest;
                WORD contentRegSRH;

                bool stateInfoRequest;
                bool regSRLrequest;
                WORD contentRegSRL;

                //WORD contentRegSRL; non acceduto da getAlarms

                bool actuatorIDInMotion;
                bool powerOffCommand;
                int internalHomingStateDefault; // N.B. Per ragioni di efficienza questo membro e' utile che rimanga pubblico
                int internalHomingStateHoming2; // N.B. Per ragioni di efficienza questo membro e' utile che rimanga pubblico

                bool readyState;

                bool LSNactive;
                bool LSPactive;
               
                double voltage_LNS; //[V]
                double voltage_LPS; //[V]
                double range;
                double fullScalePot;
                double constantPot;

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
//                typedef boost::shared_ptr< SerialCommChannelTechnosoft > channel_psh;
//                channel_psh my_channel;
//                typedef std::map<std::string,channel_psh> channel_map_t;

                //static channel_map_t channels; // gli oggetti TechnoSoftLowDriver condivideranno una struttura dati map,
                                              // percio e dichiarata di tipo static

                //bool alreadyopenedChannel;  // Canale di comunicazione aperto
                //bool poweron; // alimentazione al drive motor erogata
                //bool channelJustOpened;

                //void* incrDecrPosition(void* arg);

            public:
                // Costruttore device channel and device name
                TechnoSoftLowDriver();
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
                        const double _linear_movement_per_n_rounds=LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT,
                        const double voltage_LNS = V_LNS, //[V]
                        const double voltage_LPS = V_LSP, //[V]
                        const double range = RANGE,  //[meter]
                        const double fullScalePot = FULLSCALE_POTENTIOMETER, //[V]
                        const int _alarmsPresent = ALARMS_PRESENT_DEFAULT,
                        const double _alarmsInterval = DURATION_ALARMS_INTERVAL_DEFAULT,
                        const double _percOfnoise = PERC_NOISE_DEFAULT,
                        const double _probabilityError = PROBABILITY_OPERATION_ERROR
                        );


                int homing(int mode);
                int soft_homing(double positionToSet);
                int getinternalHomingStateDefault();
                int getinternalHomingStateHoming2();
		double speedfromMMsToIU(double _speed_mm_s);
 		double speedfromIUTOMMs(double _speed_IU);
		double accelerationfromMMs2ToIU(double _acceleration_mm_s2);
		double accelerationfromIUToMMs2(double _acceleration_IU);

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
                
                int setRatiOfNoise(const double& _ratiOfNoise);
                
                int setMaxSpeed(const double& maxspeed);
                int getMaxSpeed(double& maxspeed);
                
                int setAcceleration(const double& acceleration);
                int getAcceleration(double& acceleration);
                
		int setMeasureUnit(const bool& inSteps);
                int getMeasureUnit(bool& inSteps);

                int setMaxAcceleration(const double& maxAcceleration);
                int getMaxAcceleration(double& maxAcceleration);
                
                int setAdditive(const BOOL& isAdditive);
                int getAdditive(BOOL& isAdditive);
                
                int setMovement(const short& movement);
                int getMovement(short& movement);
                
                int setReferenceBase(const short& referenceBase);
                int getReferenceBase(short& referenceBase);
                
                int setAlarmsGeneration(const int& intValue);
                int getAlarmsGeneration(int& intValue);

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
                
                int hardreset(bool mode=HARD_RESET_MODE_DEFAULT); 
                
//                int setAdditiveHoming(const BOOL& isAdditive);
//                int setMovementHoming(const short& movement);
//                int setReferenceBaseHoming(const short& referenceBase);

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
                
                int setvoltage_LNS(const double& _voltage_LNS);
                int getvoltage_LNS(double& _voltage_LNS);
                
                int setvoltage_LPS(const double& _voltage_LPS);
                int getvoltage_LPS(double& _voltage_LNS);
                
                int setRange(const double& _range);
                int getRange(double& _range);
                
                int setFullscalePot(const double& _fullScale);
                int getFullscalePot(double& _fullScale);
                
                int setAlarmsInterval(const double& _value);
                int getAlarmsInterval(double& _value);
                
                int setPercOfNoise(const double& value);
                int getPercOfNoise(double& value);
                
                int setProbError(const double& value);
                int getProbError(double& Value);
                
                //Encoder lines
                int moveAbsoluteSteps(const long& position);
                int moveAbsoluteStepsHoming(const long& position);
                
                int getPotentiometer(double* deltaPosition_mm);

                // get methods for variables
                //channel_psh getMyChannel();
                int getCounter(double* deltaPosition_mm);
                int getEncoder(double* deltaPosition_mm);
                // resetting methos
                //int resetCounter();// reset TPOS_register();
                int resetCounterHoming();
                int resetEncoderHoming();// reset APOS_register();
//                static void* staticResetCounterForThread(void*);
                static void* staticResetEncoderForThread(void*);


                int resetFault();
                int resetFaultAlarms();
                static void* staticResetFaultFunctionForThread(void* objPointer);
                
                int resetSetup();
                int getPower(BOOL& powered); //***************** Questo metodo dovrà essere sostituito da:
                //int getRegister()**********************;
                int stopMotion();
                static void* staticStopMotionForThread(void* objPointer);
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
}

#endif /* TECHNOSOFTLOWDRIVER_H */

