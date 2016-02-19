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
#define SPEED 400.0 // 30.0
#define ACCELERATION 0.1 // 0.6


#define MAX_LENGTH_STRING_FROM_SHELL 50
#define MAX_COMMAND_LENGTH 10

            
#define CONST_MULT_TECHNOFT 256.0 // numero micro steps per step (MDS)
#define STEPS_PER_ROUNDS 200.0     // numero steps per giro
#define N_ROUNDS 20.0 
#define LINEAR_MOVEMENT_PER_N_ROUNDS 1.5 //[mm]
#define N_ENCODER_LINES 800.0 //[mm]
#define RANGE 20.0  
#include <map>
#include <boost/shared_ptr.hpp>

namespace common{
    
    namespace actuators{
       namespace models {

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
                
                    ~SerialCommChannelTechnosoft();
                    BOOL open(int hostID=HOST_ID);
                    void close();
            };
        
            //TechnoSoftLowDriver class
            class TechnoSoftLowDriver {

	    private:
                int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
                int axisRef;// handler
                
                std::string dev;
                std::string devName;
                
                // Trapezoidal profile parameters
                //long relPosition;
                double speed;
                double acceleration;
                bool isAdditive;
                short movement;
                short referenceBase;
                // Additional parameters for s-curve profile
                long jerkTime;
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
                
            public:
                // Costruttore device channel and device name
                TechnoSoftLowDriver(const std::string dev, const std::string devName);
                // *************ATTENZIONE, DICHIARARE IL METODO DISTRUTTORE******************** 
                ~TechnoSoftLowDriver();
                // Inizializzazione singolo drive/motor
                int init(const std::string& setupFilePath,const int& axisID, const double speed=SPEED, const double acceleration=ACCELERATION, const BOOL isAdditive=0, const short moveMoment =UPDATE_IMMEDIATE, const short referenceBase=FROM_REFERENCE, const int encoderLines=N_ENCODER_LINES);
                
                //LONG RelPosition, DOUBLE Speed, DOUBLE Acceleration, BOOL IsAdditive, SHORT MoveMoment, SHORT ReferenceBase)
                //void setupTrapezoidalProfile(long, double, double, BOOL, short, short);
                int providePower();
                int stopPower();
                BOOL moveRelativeSteps(const long& deltaPosition);// (0 -> OK)  (different 0 -> error)
                BOOL moveAbsoluteSteps(const long& position);
                
                // get methods for variables
                BOOL getCounter(long& tposition);
                BOOL getEncoder(long& aposition);
                // resetting methos
                BOOL resetCounter();// reset TPOS_register();
                BOOL resetEncoder();// reset APOS_register();
                BOOL getPower(BOOL& powered); //***************** Questo metodo dovrà essere sostituito da:
                //int getRegister()**********************;
                BOOL stopMotion();
                int deinit();
                BOOL setDecelerationParam(double deceleration);
                BOOL setFixedVariable(LPCSTR pszName, double value);
                BOOL abortNativeOperation();
                BOOL executeTMLfunction(std::string&);
                BOOL setVariable(LPCSTR pszName, long value);
                BOOL readHomingCallReg(short selIndex, WORD& status);
                BOOL setEventOnLimitSwitch(short lswType , short transitionType, BOOL waitEvent, BOOL enableStop);
                BOOL setEventOnMotionComplete(BOOL waitEvent, BOOL enableStop);
                BOOL setPosition(const long& posValue);
                
                BOOL getLVariable(const std::string& nameVar,long* var) const;
                
                BOOL checkEvent(BOOL& event);

                int getMERregister();// REG_MER_register();
                int getSRLregister();// REG_SRL_register();
                //******************* da aggiungere la lettura dell'altro registro rimanente ******************
                
           };
	}// chiude namespace technosoft

    }
}

