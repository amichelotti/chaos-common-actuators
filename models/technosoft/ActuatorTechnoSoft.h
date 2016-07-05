//
//  ActuatorTechnoSoft.h
//  prova
//
//  Created by MacBookProINFN on 29/01/16.
//  Copyright © 2016 MacBookProINFN. All rights reserved.
//

#include <common/actuators/core/AbstractActuator.h>
#include "TechnoSoftLowDriver.h"
#include <sys/time.h>


#ifndef ActuatorTechnoSoft_h
#define ActuatorTechnoSoft_h

namespace common{
    
    namespace actuators{

	namespace models{

          class ActuatorTechnoSoft:public ::common::actuators::AbstractActuator{
    
                private:
                    TechnoSoftLowDriver *driver;
                  
                    std::string dev_name; // ActuatorTechnoSoft name
                    
                    bool readyState;
                    bool partialInit;
                    int internalHomingStateDefault;
                    int internalHomingStateHoming2;
                    
                    uint8_t btType;
                    uint32_t baudrate;
                    int hostID;
                
                public:
                    
                    // costruttore
                    ActuatorTechnoSoft();
                    // Costruttore di copia
                    ActuatorTechnoSoft(const ActuatorTechnoSoft&); // Overloading costruttore di copia
                    ActuatorTechnoSoft& operator=(const ActuatorTechnoSoft& objActuator); // Overloading operatore assegnamento 
                    ~ActuatorTechnoSoft();

                    int init(void*initialization_string);
                    int configAxis(void*initialization_string);

                    int deinit(int axisID);
                    int moveRelativeMillimeters(int axisID,double deltaMillimeters);
                    int moveVelocityHoming();
                
                    int setParameter(int axisID,std::string parName,std::string value);
                   
                    int setTrapezoidalProfile(double speed, double acceleration, bool isAdditive, int32_t movement, int32_t referenceBase);
                    int setSpeed(double speed);
                    int setMaxSpeed(double speed); //[mm/s]
                    int setAcceleration(double acceleration);
                    int setMaxAcceleration(double acceleration);
                    int setAdditive(bool isAdditive); // NOTA: bool dovra' essere castato a int
                    int setMovement(int32_t movement); // NOTA: int32_t dovra' essere castato a short
                    int setReferenceBase(int32_t referenceBase); // NOTA: int32_t dovra' essere castato a short
                
                    // Set Homing parameters
                    int sethighSpeedHoming(double speed);
                    int setMaxhighSpeedHoming(double speed);
                    int setlowSpeedHoming(double speed);
                    int setMaxlowSpeedHoming(double speed);
                    int setAccelerationHoming(double acceleration);
                    int setAdditiveHoming(bool isAdditive); // NOTA: bool dovra' essere castato a int
                    int setMovementHoming(int32_t movement); // NOTA: int32_t dovra' essere castato a short
                    int setReferenceBaseHoming(int32_t referenceBase); // NOTA: int32_t dovra' essere castato a short
                    int setMaxAccelerationHoming(double acceleration);
                
                    int setEncoderLines(double _encoderLines);
                    int setConst_mult_technsoft(double _const_mult_technsoft);
                    int setSteps_per_rounds(double _steps_per_rounds);
                    int setN_rounds(double _n_rounds);
                    int setLinear_movement_per_n_rounds(double _linear_movement_per_n_rounds);

                    int moveAbsoluteMillimeters(int axisID,double mm);
                    
                    int poweron(int axisID,int on);
                    int selectAxis();
                
                    int resetAlarms(int axisID,uint64_t alrm); 

                    int stopMotion(int axisID);
                    int getPosition(int axisID,readingTypes mode, double* deltaPosition_mm);
                    int homing(int axisID,homingType mode);
                    int getState(int axisID,int* state, std::string& desc );   // **
                    int getAlarms(int axisID,uint64_t*alrm, std::string& descStr);
                    uint64_t getFeatures(){return 0;}
     
        /**
        @brief get the actuator position using the readingType mode chosen for reading
        @return 0 if success or an error code
        */    
           
        /**
           @brief returns the SW/FW version of the driver/FW
           @param version returning string
           @return 0 if success or an error code
        */
	

	int getSWVersion(std::string& version);

        /**
        @brief returns the HW version of the actuator 
        @param version returning string
        @return 0 if success or an error code
        */
        int getHWVersion(std::string& version);
        /**
        @brief returns dataset of the driver settable with setParameter method 
        @param  dataset returning string
        @return 0 if success or an error code
        */
	
	int sendDataset(std::string& dataset);
        };
	}
    }
}
#endif /* ActuatorTechnoSoft_h */
