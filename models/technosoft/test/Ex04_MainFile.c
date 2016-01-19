
/*=================================================================================*/
/* Function: Main function in the application                                      */
/*=================================================================================*/

#include "Ex04_MainFile.h"
#include <stdlib.h>

BOOL initCommunicationChannel(int* fd)
{
	int resp;
/*	Open the comunication channel: COM1, RS232, 1, 115200 */
	if((resp=TS_OpenChannel(CHANNEL_NAME, CHANNEL_TYPE, HOST_ID, BAUDRATE)) < 0)
	{	
		return FALSE;
	}
	*fd = resp;
	return TRUE;
}

BOOL initAllAxis(int idAxis[], int nAxis)
{
/*	Load the *.t.zip with setup data generated with EasyMotion Studio or EasySetUp, for AXIS_ID_01*/
	idAxis[0] = TS_LoadSetup(SETUP_FILE_01);
	if(idAxis[0] < 0) 
		return FALSE;
/*	Load the *.t.zip with setup data generated with EasyMotion Studio or EasySetUp, for AXIS_ID_02*/
	//idAxis[1] = TS_LoadSetup(SETUP_FILE_02);
	//if(idAxis[1] < 0) 
		//return FALSE;

/*	Setup the axis based on the setup data previously, for AXIS_ID_01*/
	if(!TS_SetupAxis(AXIS_ID_01, idAxis[0])) 
		return FALSE;
/*	Setup the axis based on the setup data previously, for AXIS_ID_02*/
	//if(!TS_SetupAxis(AXIS_ID_02, idAxis[1])) 
		//return FALSE;
	
	return TRUE;
}

//WORD sAxiOn_flag = 0;
/*	Select the destination axis of the TML commands */
	//if(!TS_SelectAxis(AXIS_ID_01)) 
		//return FALSE;

/*	Execute the initialization of the drive (ENDINIT) */
	//if(!TS_DriveInitialisation()) 
		//return FALSE;

/*	Enable the power stage of the drive (AXISON) */ 
	//if(!TS_Power(POWER_ON)) 
		//return FALSE;

/*	Wait for power stage to be enabled */
	//while(sAxiOn_flag == 0)
	//{
		/* Check the status of the power stage */
		//if(!TS_ReadStatus(REG_SRL, &sAxiOn_flag)) 
			//return FALSE;
		//sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
	//}

	//return TRUE;



#if !(defined(WINDOWS) || defined(WIN32))
int getch()
{
	char c;
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
	while(fgetc(stdin) != EOF);
	if(errno == EWOULDBLOCK)
	{
		fcntl(STDIN_FILENO, F_SETFL, 0);
		read(STDIN_FILENO, &c, 1);
	}
	return c;
}
#endif



int main(int argc, char** argv)
{
	 
	/*Open the communication channel */
        int fd; // file descriptor of the communication channel
	if(!initCommunicationChannel(&fd))
	{
		fprintf(stderr, " Communication error! \n %s\n", TS_GetLastErrorText());
		return -1;
	}

	/*Setup and initialize every axis with which is possible to communicate via the same channel (just initialized)*/
	int idAxis[NAXIS]; // Vettore che contiene i descrittori degli assi
	if(!initAllAxis(idAxis,NAXIS))
	{
		fprintf(stderr, " Error noticed during the axis initialization!\n");
		TS_CloseChannel(fd);
		fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
		return -2;
	}

	/*Print on the screen the names of the initialized axis*/
	printf("It's possible to control the following slits:\n");
	char *axisNames[NAXIS]={"SLT TB 001 NERO"};
	int i = 0;
	for(i=0;i<NAXIS;i++)
		printf("%s, n. %d\n",axisNames[i],1);

	//int idSelected;
         
	//char command[MAX_COMMAND_LENGTH];  
	//char commandArg1[MAX_COMMAND_LENGTH];   	
	
    	//long deltaPosition; /* Es. 80000, position command [drive internal position units, encoder counts] */
    	//double speed; /* Es. 30, slew speed [drive internal speed units, encoder counts/slow loop sampling] */
    	//double acceleration; /*Es. 0.6, acceleration rate [drive internal acceleration units, encoder counts/slow loop sampling^2]*/
    
    	//long cposition = -1;		/* command position read from the drive [drive internal position units, encoder counts] */
    			/* actual motor position read from the drive [drive internal position units, encoder counts]*/
	

	//WORD home_status = 0;
	
	
	//char currentAxisName[MAX_NAMEAXIS_LENGTH]="";

	while (TRUE)
	{

		char str[MAX_COMMAND_LENGTH];
		char del[]="\t ";
		
		char firsArg[MAX_COMMAND_LENGTH]="";
		char secondArg[MAX_COMMAND_LENGTH]="";

		BOOL firstCorrectArgument = FALSE;

		/*int resp=-1;
		BOOL corretComand = FALSE;
		
		printf(strcat(currentAxisName,">> "));*/

		/*      Command specification from user		*/
		while(!firstCorrectArgument)
		{
			printf(">> ");
			strcpy(str,"");
			int resp = scanf("%[^\n]%*c",str);//*c%[^\n]%
			//printf(str);
			//printf("%d\n",strlen(str));
			int cont =0;
			if(resp==1){
				//Analizziamo la stringa inserita:
				char* p;
				for(p=strtok(str,del);p!=NULL;p=strtok(NULL,del))
				{
					cont++;
					if(cont==1 && (strcmp(p,"AXID")==0 || strcmp(p,"MOVES")==0 || strcmp(p,"STOP")==0 || strcmp(p,"AXIS")==0 || 
					strcmp(p,"RESE")==0 || strcmp(p,"COUT")==0 || strcmp(p,"ENCO")==0 || 
					strcmp(p,"FIN")==0 || strcmp(p,"FOUT")==0 || strcmp(p,"EM")==0 || 
					strcmp(p,"ESCO")==0 || strcmp(p,"INFO")==0 || strcmp(p,"OVER")==0 || strcmp(p,"I2t")==0))
					{	
						firstCorrectArgument=TRUE;
						strcpy(firsArg,p);
					}
					else if(cont==1)
					{
						printf(firsArg);
						printf("The command specified doesn't exist.\n");
						break;
					}
					else if(cont==2)
					{
						strcpy(secondArg,p);
						break;
					}
				}
			}
			else
			{
				printf("No command has been specified correctly.\n");
			}
			if(cont==1 && firstCorrectArgument)
			{
				break;
			}
			
		}
		if(strcmp(firsArg,"AXID")==0)
		{ 
			/*while(resp1<=0 || resp1==EOF ) // Ci andra un controllo sugli indici corretti che l`utente puo inserire
			{
  				printf("\nSelect the slit for motion or to know its status by specifying its number: \n");
				resp1=scanf("%d",&idSelected);
				// Nel caso generale occorre inserire una procedura che, assegnato ad esempio a idSelected il n. 1, vada ad assegnare alla variabile 
                        	// idSelected l'id Axis
			
			}
			//resp1=-1;
			printf("\nB");*/
			/*	Select the destination axis of the TML commands */
			//int idSelected = 14;	
			//printf("%d\n",atoi(secondArg));
			if(!TS_SelectAxis(atoi(secondArg))) // idSelected=14......................................
			{ 
				fprintf(stderr,"Error noticed during the execution of the selection procedure of the axis.\n");
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -3;
			}
			//printf("\nC");
			/*	Execute the initialization of the drive (ENDINIT) */
			if(!TS_DriveInitialisation())
			{ 
				fprintf(stderr,"Drive initialization error.\n");
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -4;
			}
			//printf("\nD");
			//currentAxisName[0]='0';
			//strcpy(currentAxisName,"SLT TB 001 NERO");
		}
		else if(strcmp(firsArg,"MOVES")==0)
		{
			//printf("ci sto dentro");
			long deltaPosition=atol(secondArg); // da cambiareeeeeee...in futuro **********************************************************
			deltaPosition*=CONST_MULT;
			//printf("%ld",deltaPosition);
			if(!TS_MoveRelative(deltaPosition, SPEED, ACCELERATION, 0,UPDATE_IMMEDIATE,FROM_REFERENCE))// 0==false
			{ 
                    		fprintf(stderr,"%s\n",TS_GetLastErrorText());
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
                    		return -5;
			}
                }
		else if(strcmp(firsArg,"STOP")==0)
		{ 
			// The function stops the motor with the deceleration rate set in TML parameter CACC. For further informations see documentation
			if(!TS_Stop())
			{ 
                    		fprintf(stderr,"%s\n",TS_GetLastErrorText());
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
                    		return -6;
                	}			
		}
		else if(strcmp(firsArg,"AXIS")==0)
		{ 
			//printf("%s\n",command);
			//printf("%s\n",commandArg1);
			if(strcmp(secondArg,"ON")==0)
			{
				//printf("comando power onnnnnnn1");
				/*	Enable the power stage of the drive (AXISON) */ 
				if(!TS_Power(POWER_ON))
				{  	
					fprintf(stderr,"Error caused by the enable of the power stage of the current axis.\n");
					TS_CloseChannel(fd);
					fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n",fd);
					return -7;
				}
				/*	Wait for power stage to be enabled */
				WORD sAxiOn_flag = 0;
				while(sAxiOn_flag == 0)
				{	
					/* Check the status of the power stage */
					if(!TS_ReadStatus(REG_SRL, &sAxiOn_flag)) 
					{
						fprintf(stderr,"Status information reading error.\n");
						TS_CloseChannel(fd);
						fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
						return -8;
					}
					sAxiOn_flag=((sAxiOn_flag & 1<<15) != 0 ? 1 : 0);
				}
			}
			else if(strcmp(secondArg,"OFF")==0)
			{
				//printf("Comando power offffffffff");
				/*	Disable the power stage of the drive (AXISOFF) */
				if(!TS_Power(POWER_OFF))
				{
					fprintf(stderr,"Error caused by the disable of the power stage of the current axis.\n");
					TS_CloseChannel(fd);
					fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
					return -9;
				}
			}
			else
			{
				fprintf(stderr,"No actions performed. Parameter of the %s command is wrong!\n",firsArg);
				return -10;
			}
		}
		else if(strcmp(firsArg,"RESE")==0) 
		{
			if(strcmp(secondArg,"T")==0)
			{	
				if(!TS_Execute("SAP 0"))
				{
					fprintf(stderr,"Error caused by reset operation of TPOS register.\n");
					TS_CloseChannel(fd);
					fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);					
					return -11;
				}
			}
			else if(strcmp(secondArg,"A")==0)
			{
				if(!TS_Execute("APOS=0"))
				{
					fprintf(stderr,"Error caused by reset operation of APOS register.\n");
					TS_CloseChannel(fd);
					fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);	
					return -12;
				}
			}
			else
			{
				fprintf(stderr,"No actions performed. Parameter of the %s command is wrong!\n",firsArg);
				return -13;
			}
		}
		/*else if(strcmp(command,"HOME")==0 && resp2==2){
			if(strcmp(commandArg1,"1"))
			{
			}
			else if(strcmp(commandArg1,"2"))
			{
			}
			else{
			//	errore inserimento valore secondo parametro     
			}
		}*/
		// Comandi per la lettura dei valori in determinati registri del driver:

		// Lettura dei valori del registro TPOS (counter)
		else if(strcmp(firsArg,"COUT")==0)
		{
			long tposition;
			if(!TS_GetLongVariable("TPOS", &tposition))
			{
	                    	fprintf(stderr,"%s\n",TS_GetLastErrorText());
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
                    		return -17;
                	}
			printf("The current position reference is %ld.\n\n",tposition/CONST_MULT);
		}
		// Lettura valore registro APOS (counter)
        	else if(strcmp(firsArg,"ENCO")==0)
		{
			long aposition;
		 	if(!TS_GetLongVariable("APOS", &aposition))
			{
	                    	fprintf(stderr,"%s\n",TS_GetLastErrorText());
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
	                    	return -18;
	                }
			printf("The actual position from the drive is %ld.\n\n",aposition);
		}
		// Check fine corsa IN
		else if(strcmp(firsArg,"FIN")==0)
		{
			WORD axisIn; 
			if(!TS_ReadStatus(REG_MER, &axisIn)) 
			{ 
				fprintf(stderr,"Status information reading error.\n");
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -19;
			}
			axisIn = ((axisIn & 1<<6)==0 ? 1 : 0);  // Non dovrebbe essere 7 o 6? ***************************
			if(axisIn != 0)
				printf("Closed - normal operation\n");
			else
				printf("Opened - engaged\n");		
		}
		// Check fine corsa OUT
		else if(strcmp(firsArg,"FOUT")==0)
		{
			WORD axisOut;
			if(!TS_ReadStatus(REG_MER, &axisOut))
			{ 
				fprintf(stderr,"Status information reading error.\n");
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -20;
			}
			axisOut = ((axisOut & 1<<7)==0 ? 1 : 0); // Non dovrebbe essere 7 o 6? ***************************
			if(axisOut != 0)
				printf("Closed - normal operation\n");
			else
				printf("Opened - engaged\n");		
		}
		// Check if there is over current
		else if(strcmp(firsArg,"OVER")==0)
		{
			WORD overCurrent;
			if(!TS_ReadStatus(REG_MER, &overCurrent))
			{ 
				fprintf(stderr,"Status information reading error.\n");
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -21;
			}
			overCurrent = ((overCurrent & 1<<8)==0 ? 1 : 0);
			if(overCurrent == 0)
				printf("ATTENTION! Over current is occurred!\n");
			else
				printf("No over current - normal operation\n");		
		}
		else if(strcmp(firsArg,"I2t")==0)
		{
			WORD i2T;
			if(!TS_ReadStatus(REG_MER, &i2T))
			{
				fprintf(stderr,"Status information reading error.\n\n"); 
				return -22;
			}
			i2T = ((i2T & 1<<9)==0 ? 1 : 0);
			if(i2T == 0)
				printf("ATTENTION! i2t is occurred\n");      
			else
				printf("No i2t - normal operation\n");		
		}
		else if(strcmp(firsArg,"INFO")==0) 
		{
			WORD power = -1;
			//printf("Comando INFO inviato\n");
			if(!TS_ReadStatus(REG_SRL, &power))
			{
				fprintf(stderr,"Status information reading error.\n"); 
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -22;
			}
			power = ((power & 1<<15)==0 ? 1 : 0);
			if(power != 1)
				printf("Axis is powered\n");      
			else
				printf("Axis is NOT powered\n"); 	
		}
		/* Check the status of the digital input port 16 (EMERGENCY)*/
		else if(strcmp(firsArg,"EM")==0)
		{
			BYTE emPortN=16;
			BYTE emValue=0;

			if(!TS_GetInput(emPortN, &emValue))
			{ 
				fprintf(stderr,"Error caused by the reading operation of the status of the digital port number 16 .\n");
				TS_CloseChannel(fd);
				fprintf(stderr, " The communication channel, whose fd=%d has been closed.\n\n",fd);
				return -24;
			}
			printf("The emergency status is %d\n",emValue); // Dovrebbe essere letta cosi la variabile emValue*************************************
		}
		
		/*	Close the communication channel */
		else // The specified command is ESCO
		{
			TS_CloseChannel(fd);
			break;
		}
		
	}// chiude while(true)
	return 0;
}
