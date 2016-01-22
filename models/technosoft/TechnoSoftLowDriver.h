#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <string>

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
int getch();
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
#define CONST_MULT 256
#define MAX_NAMEAXIS_LENGTH 256

#define MAX_LENGTH_STRING_FROM_SHELL 50
#define MAX_COMMAND_LENGTH 10

#if defined(WINDOWS) || defined(WIN32)
#define SETUP_FILE_01 "Exx_setup_file_ID1.t.zip"
#else
//#define SETUP_FILE_01 "../../TML_LIB_User/Exx_setup_file_ID1.t.zip"
#define SETUP_FILE_01 "/u2/dcs/prefs/MOV/setups/1setup001.t.zip"
#define SETUP_FILE_02 "/u2/dcs/prefs/MOV/setups/1setup001.t.zip"
#endif
            
#define CONST_MULT_TECHNOFT 256


///#include <common/debug/core/debug.h>
namespace common{
    
    namespace actuators{
        namespace technosoft{

	    //Channel class
            class SerialCommChannelTechnosoft{
                
                private:
                    char pszDevName[100];
                    BYTE btType;
                    DWORD baudrate;
                    int fd;
                public:
                    SerialCommChannelTechnosoft(){};
		    ~SerialCommChannelTechnosoft(){}
		    // *************ATTENZIONE, DICHIARARE IL METODO DISTRUTTORE******************** 
                    void init(const std::string& pszDevName,const BYTE& btType,const DWORD& baudrate);
                    BOOL open(int hostID);
                    void deinit();
            };
        
            //TechnoSoftLowDriver class
            class TechnoSoftLowDriver {
                
            public:
                // Costruttore
                TechnoSoftLowDriver(const std::string&);
                // *************ATTENZIONE, DICHIARARE IL METODO DISTRUTTORE******************** 
		~TechnoSoftLowDriver(){}
                // Inizializzazione singolo drive/motor
                int init(const int&, const double&, const double&, const BOOL&, const short&, const short&);
                
                //LONG RelPosition, DOUBLE Speed, DOUBLE Acceleration, BOOL IsAdditive, SHORT MoveMoment, SHORT ReferenceBase)
                //void setupTrapezoidalProfile(long, double, double, BOOL, short, short);
                int providePower();
                int stopPower();
                BOOL moveRelativeSteps(long);// (0 -> OK)  (≠0 -> error)
                // get methods for variables
                BOOL getCounter(long&);
                BOOL getEncoder(long&);
                // resetting methos
                BOOL resetCounter();// reset TPOS_register();
                BOOL resetEncoder();// reset APOS_register();
                BOOL getPower(BOOL&); //***************** Questo metodo dovrà essere sostituito da:
                //int getRegister()**********************;
                BOOL stopMotion();
                int deinit();
                
                int getMERregister();// REG_MER_register();
                int getSRLregister();// REG_SRL_register();
                //******************* da aggiungere la lettura dell'altro registro rimanente ******************
                
            private:
                int axisID;// numero dell’asse (selezionabile da dip switch su modulo Technosoft
                int axisRef;// handler
                
                // Trapezoidal profile parameters
                long relPosition;
                double speed;
                double acceleration;
                bool isAdditive;
                short movement;
                short referenceBase;
                // Additional parameters for s-curve profile
                //long jerkTime;
                //short decelerationType;
                
                char setupFilePath[200];
                
                // ************** cosa rappresentano queste tre variabili? ***************
                int absoluteSteps;// contatore software
                int status_register;// Reg_MER
                int error_register; 
            };
}}}
