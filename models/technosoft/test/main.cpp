
/*=================================================================================*/
/* Function: Main function in the application                                      */
/*=================================================================================*/

#include <common/actuators/models/technosoft/TechnoSoftLowDriver.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
  std::string path=".";
  common::actuators::technosoft::TechnoSoftLowDriver tc(path);
    
	return 0;
}
