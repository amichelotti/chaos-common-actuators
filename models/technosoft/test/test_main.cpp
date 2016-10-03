#include "iostream"
#include "TechnoSoftLowDriver.h"

int main(int argc,const char* argv[]){
	
	// Inizializzazione canale seriale
	common::actuators::technosoft::SerialCommChannelTechnosoft serial1;
	std::string channelName = CHANNEL_NAME;
	BYTE btType = HOST_ID;
	DWORD baudrate = BAUDRATE;
	serial1.init(channelName,btType,baudrate);

	// Apertura canale
	if(!serial1.open(HOST_ID))
		return -1;

	// Costruzione oggetto TechnoSoftLowDriver1
	common::actuators::technosoft::TechnoSoftLowDriver techSoftDriveMotor1(SETUP_FILE_01);
	
	// Inizializzazione techSoftDriveMotor1
	int axisID = AXIS_ID_01;
	double speed = SPEED;
	double acceleration = ACCELERATION;
	BOOL isAdditive = 	FALSE;
	short moveMoment = UPDATE_IMMEDIATE;
	short referenceBase = FROM_REFERENCE;
	techSoftDriveMotor1.init(axisID, speed, acceleration, isAdditive, moveMoment, referenceBase);

	// Reset encoder di techSoftDriveMotor1
	if(!techSoftDriveMotor1.resetEncoder())
		return -1;

	// Reset counter di techSoftDriveMotor1
	if(!techSoftDriveMotor1.resetCounter())
		return -2;

	// Lettura encoder di techSoftDriveMotor1 (PRIMA della movimentazione)
	long apositionFirst;
	if(!techSoftDriveMotor1.getEncoder(apositionFirst)){
		return -3;
	}
	else
		std::cout<<"The value of encoder is "<< apositionFirst<<std::endl;

	// Lettura counter di techSoftDriveMotor1 (PRIMA della movimentazione)
	long tpositionFirst;
	if(!techSoftDriveMotor1.getCounter(tpositionFirst)){
		return -4;
	}
	else
		std::cout<<"The value of counter is "<<tpositionFirst<<std::endl;

	
	// Alimentazione techSoftDriveMotor1
	// nota: non e' necessario erogare alimentazione per leggere lo stato del dispositivo (utilizzando i metodi get)
	if(techSoftDriveMotor1.providePower()<0)
		return -5;

	// Movimentazione techSoftDriveMotor1	
	long relPosition;
	std::cout<<"Enter number of steps for motion: " << std::endl;
	std::cin>>relPosition;
	if(!techSoftDriveMotor1.moveRelativeSteps(relPosition))
		return -6;

	sleep(10);

	// Stop movimentazione techSoftDriveMotor1
	if(!techSoftDriveMotor1.stopMotion())
		return -7;

	// Lettura encoder di techSoftDriveMotor1 (DOPO la movimentazione)
	long apositionAfter;
	if(!techSoftDriveMotor1.getEncoder(apositionAfter)){
		return -8;
	}
	else
		std::cout<<"The value of encoder is "<< apositionAfter<<std::endl;

	// Lettura counter di techSoftDriveMotor1 (DOPO la movimentazione)
	long tpositionAfter;
	if(!techSoftDriveMotor1.getCounter(tpositionAfter)){
		return -9;
	}
	else
		std::cout<<"The value of counter is "<<tpositionAfter<<std::endl;

	// Check alimentazione techSoftDriveMotor1
	BOOL poweredFirst;
	if(!techSoftDriveMotor1.getPower(poweredFirst))
		return -10;
	else
		std::cout<<"The power state is: "<< poweredFirst << std::endl;

	// Interruzione alimentazione techSoftDriveMotor1
	techSoftDriveMotor1.stopPower();

	// Check alimentazione dopo aver tagliato l'alimentazione a techSoftDriveMotor1
	BOOL poweredAfter;
	if(!techSoftDriveMotor1.getPower(poweredAfter))
		return 11;
	else
		std::cout<<"Now, the power state is: "<< poweredAfter << std::endl;

	// deinizializzazione techSoftDriveMotor1: di nuovo interruzione alimentazione techSoftDriveMotor1	
	techSoftDriveMotor1.deinit(); // non fa niente...
	
	// Deinizializzazione canale: chiusura canale di comunicazione
	serial1.deinit();

	return 0;
}
