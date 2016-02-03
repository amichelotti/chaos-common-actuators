#include <iostream>
#include "TechnoSoftLowDriver.h"

int main(int argc,const char* argv[]){
	
	// Inizializzazione canale seriale
	common::actuators::technosoft::SerialCommChannelTechnosoft serial1;
	std::string channelName = CHANNEL_NAME;
	BYTE btType = HOST_ID;
	DWORD baudrate = BAUDRATE;
	serial1.init(channelName,btType,baudrate);

	// Apertura canale
	if(!serial1.open(HOST_ID)){
		std::cout<<"Errore apertura canale di comunicazione"<<std::endl;
		return -1;
	}

	// Costruzione oggetto Actuator
	//common::actuators::technosoft::TechnoSoftLowDriver techSoftDriveMotor1(SETUP_FILE_01);
	std::string actuator_name = "SLT TB 001";
	common::actuators::Actuator ActuatorTechSoft(actuator_name,SETUP_FILE_01);
	
	// Inizializzazione Actuator
	int axisID = AXIS_ID_01;
	double speed = SPEED;
	double acceleration = ACCELERATION;
	BOOL isAdditive = 	FALSE;
	short moveMoment = UPDATE_IMMEDIATE;
	short referenceBase = FROM_REFERENCE;
	//techSoftDriveMotor1.initTechnoSoftLowDriver(axisID, speed, acceleration, isAdditive, moveMoment, referenceBase);
	double range = RANGE; //mechanical range of the slit (passato da MDS). Vallore passato solo per il test, ma ora non viene utilizzato..
	double mechanicalReduceFactor = N_ROUNDS; // (passato da MDS), 1/x, x giri : 1 un_mov_mm
	double movementUnit_mm = LINEAR_MOVEMENT_PER_N_ROUNDS; // 1.5 mm or 1 mm (MDS) 
	int encoderLines = N_ENCODER_LINES; // (passato da MDS)
	ActuatorTechSoft.initActuator(range,mechanicalReduceFactor,movementUnit_mm,encoderLines,axisID, speed, acceleration, isAdditive, moveMoment, referenceBase);

	// Reset encoder di Actuator
	if(!ActuatorTechSoft.resetEncoder()){
		std::cout<<"Errore reset encoder"<<std::endl;
		return -1;
	}

	// Reset counter di Actuator
	if(!ActuatorTechSoft.resetCounter()){
		std::cout<<"Errore reset counter"<<std::endl;
		return -2;
	}

	// Lettura encoder di Actuator (PRIMA della movimentazione)
	double apositionFirst;
	if(ActuatorTechSoft.getPosition(FALSE,apositionFirst)!=0){
		std::cout<<"Errore get position from encoder"<<std::endl;
		return -3;
	}
	else
		std::cout<<"Shift in millimeters (read from encoder): "<< apositionFirst<<std::endl;

	// Lettura counter di Actuator (PRIMA della movimentazione)
	double tpositionFirst;
	if(ActuatorTechSoft.getPosition(TRUE,tpositionFirst)!=0){
		std::cout<<"Errore get position from counter"<<std::endl;
		return -4;
	}
	else
		std::cout<<"Shift in millimeters (read from counter): "<<tpositionFirst<<std::endl;

	
	// Alimentazione Actuator
	// nota: non e' necessario erogare alimentazione per leggere lo stato del dispositivo (utilizzando i metodi get)
	if(ActuatorTechSoft.providePower()<0)
		return -5;

	
	// Movimentazione Actuator	
	double relPosition;
	std::cout<<"Enter number of millimeters for motion: " << std::endl;
	std::cin>>relPosition;
	if(ActuatorTechSoft.moveRelativeMillimeters(relPosition)!=0){
		std::cout<<"Errore movimentazione relativa in millimetri: " << std::endl;
		return -6;
	}


	sleep(10);

	// Stop movimentazione Actuator
	if(!ActuatorTechSoft.stopMotion()){
		std::cout<<"Errore stop movimentazione: " << std::endl;
		return -7;
	}

	// Lettura encoder di Actuator (DOPO la movimentazione)
	double apositionAfter;
	if(ActuatorTechSoft.getPosition(FALSE,apositionAfter)!=0){
		return -8;
	}
	else
		std::cout<<"Shift in millimeters (read from encoder): "<< apositionAfter<<std::endl;

	// Lettura counter di Actuator (DOPO la movimentazione)
	double tpositionAfter;
	if(ActuatorTechSoft.getPosition(TRUE,tpositionAfter)!=0){
		return -9;
	}
	else
		std::cout<<"Shift in millimeters (read from counter): "<<tpositionAfter<<std::endl;

	// Check alimentazione Actuator
	BOOL poweredFirst;
	if(!ActuatorTechSoft.getPower(poweredFirst))
		return -10;
	else
		std::cout<<"The power state is: "<< poweredFirst << std::endl;


	// Interruzione alimentazione Actuator
	ActuatorTechSoft.stopPower();

	// Check alimentazione dopo aver tagliato l'alimentazione a Actuator
	BOOL poweredAfter;
	if(!ActuatorTechSoft.getPower(poweredAfter))
		return 11;
	else
		std::cout<<"Now, the power state is: "<< poweredAfter << std::endl;

	// deinizializzazione Actuator: di nuovo interruzione alimentazione techSoftDriveMotor1	
	ActuatorTechSoft.deinit(); // non fa niente in questo momento in pratica...
	
	// Deinizializzazione canale: chiusura canale di comunicazione
	serial1.deinit();

	return 0;
}
