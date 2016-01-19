#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/timeb.h>


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
#	include <TML_lib.h>
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

