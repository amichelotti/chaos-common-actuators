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
            namespace simul{

          class ActuatorTechnoSoft:public ::common::actuators::AbstractActuator{

                private:
                    //TechnoSoftLowDriver *driver;

                    std::string dev_name; // ActuatorTechnoSoft name

                    //bool readyState;
                    bool initChannelAlreadyDone;
                    //bool configAxisAlreadyDone;
                    bool delectingActuator;
                    //int internalHomingStateDefault;
                    //int internalHomingStateHoming2;

                    uint8_t btType;
                    uint32_t baudrate;
                    int hostID;
                    SerialCommChannelTechnosoft *channel;

                    std::map<int,TechnoSoftLowDriver *> motors;
                    
                    pthread_mutex_t mu;

          public:
                    
                    // costruttore
                    ActuatorTechnoSoft();
                    // Costruttore di copia
                    ActuatorTechnoSoft(const ActuatorTechnoSoft&); // Overloading costruttore di copia
                    ActuatorTechnoSoft& operator=(const ActuatorTechnoSoft& objActuator); // Overloading operatore assegnamento
                    ~ActuatorTechnoSoft();

                    int init(void*initialization_string);
                    int configAxis(void*initialization_string);

                    int deinit(int axisID); // Rimane da gestire la fase di deinit
                    int moveRelativeMillimeters(int axisID,double deltaMillimeters); // Non e' relativa a ciascun axis ID

                    int setParameter(int axisID,std::string parName,std::string value); // Non e' relativa a ciascun axis ID
                    
                    int getParameter(int axisID,std::string parName,std::string& resultString);
  
                    int moveAbsoluteMillimeters(int axisID,double mm);                       // *******OK********

                    int poweron(int axisID,int on);                                          // *******OK********
                    //int selectAxis(); // Non e' relativa a ciascun axis ID

                    int resetAlarms(int axisID,uint64_t alrm);
                    
                    int hardreset(int axisID, bool mode);

                    int stopMotion(int axisID);                                              // *******OK********
                    int getPosition(int axisID,readingTypes mode, double* deltaPosition_mm); // *******OK********
                    int homing(int axisID,homingType mode);                                  // ************* GESTIONE HOMING **************
                    int getState(int axisID,int* state, std::string& desc );                 // *******OK********
                    int getAlarms(int axisID,uint64_t*alrm, std::string& descStr);           // *******OK********
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


int getSWVersion(int axisID, std::string& version);

        /**
        @brief returns the HW version of the actuator
        @param version returning string
        @return 0 if success or an error code
        */
        int getHWVersion(int axisID, std::string& version);
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
}
#endif /* ActuatorTechnoSoft_h */