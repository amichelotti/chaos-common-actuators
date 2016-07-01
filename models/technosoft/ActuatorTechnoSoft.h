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

//#define CONST_MULT_TECHNOFT_DEFAULT 256.0 // numero micro steps per step (MDS)
//#define STEPS_PER_ROUNDS_DEFAULT 200.0     // numero steps per giro
//#define N_ROUNDS_DEFAULT 20.0 
//#define LINEAR_MOVEMENT_PER_N_ROUNDS_DEFAULT 1.5 //[mm]                                              (da MDS)
//#define RANGE_MM_DEFAULT 100.0


#ifndef ActuatorTechnoSoft_h
#define ActuatorTechnoSoft_h

namespace common{
    
    namespace actuators{

	namespace models{

          class ActuatorTechnoSoft: public ::common::actuators::AbstractActuator{
    
                private:
                    TechnoSoftLowDriver *driver;
                    //std::string dev; // Serial channel name
                    std::string dev_name; // ActuatorTechnoSoft name
                    //double movementUnit_mm; // 1.5 mm or 1 mm (MDS) 
		    //double mechanicalReduceFactor; // fattore di riduzione albero motore/slitta
                    bool readyState;
                    bool partialInit;
                    int internalHomingStateDefault;
                    int internalHomingStateHoming2;
                    //bool eventOnMotionCompleteSet;
                    //bool state0activated;
                    //double highSpeedHoming; // The homing travel speed
                    //double range;
                    uint8_t btType;
                    uint32_t baudrate;
                    int hostID;
              
                public:
//                    typedef struct __technoinfo {
//                        //const double range;
//                        const double mechanicalReduceFactor;
//                        const double movementUnit_mm;
//                        const int encoderLines;
//                        // SerialCommChannelTechnosoft parameters:
//                        const std::string pszDevName;
//                        const BYTE btType;
//                        const DWORD baudrate;
//                    } technoinfo_t;
                    
                // costruttore
                ActuatorTechnoSoft();
                // Costruttore di copia
                ActuatorTechnoSoft(const ActuatorTechnoSoft&); // Overloading costruttore di copia
                ActuatorTechnoSoft& operator=(const ActuatorTechnoSoft& objActuator); // Overloading operatore assegnamento 
                ~ActuatorTechnoSoft();

                int init(void*initialization_string);
                int configAxis(void*initialization_string);
        // OK
        // all'interno di initActuator dovra essere richiamata la funzione initTechnoSoft
                int deinit();
                int moveRelativeMillimeters(double deltaMillimeters);
                //int moveRelativeMillimetersHoming(double deltaMillimeters);
                int moveVelocityHoming();
                
                int setParameter(std::string parName,std::string value);
                   
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

                int moveAbsoluteMillimeters(double mm);
                //int moveAbsoluteMillimetersHoming(double mm);
                
                int poweron(int on);
                int selectAxis();
                
                // ************ Function for FAULT state reset: to be used when SRH.15 = 1 *******************  
                // Set to 0 quasi tutti i bit del registro MER (tranne MER.15, MER.7, MER.6, MER.2),
                // - the Ready ouput (if present) is set to ready level;
                // - the Error output (if present) is set to no error level 
                //and the drive motor returns to normal operation.
                int resetAlarms(uint64_t alrm); 
                
                //int getPosition(readingTypes mode, double& deltaPosition_mm);
                int stopMotion();
                int getPosition(readingTypes mode, double* deltaPosition_mm);
                int homing(homingType mode);
                int getState(int* state, std::string& desc );   // **
                int getAlarms(uint64_t*alrm, std::string& descStr);
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
