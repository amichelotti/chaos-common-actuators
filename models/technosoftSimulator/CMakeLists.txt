cmake_minimum_required(VERSION 2.9)
project(common_actuators_technosoftSimulator)
find_package(chaos)
ADD_DEFINITIONS(-DDEBUG -fpic)

FILE(GLOB src *.cpp)    
ADD_LIBRARY(${PROJECT_NAME} STATIC ${src} )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_debug  boost_log boost_system boost_thread pthread chaos_common )

if(COMMON_TEST)
ADD_EXECUTABLE(test_technosoftSim_slit test/test_slit.cpp)
TARGET_LINK_LIBRARIES(test_technosoftSim_slit ${PROJECT_NAME} chaos_common common_actuators common_debug ${chaoslib} ${FrameworkLib} )

INSTALL(TARGETS test_technosoftSim_slit
        DESTINATION "bin"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
ENDIF()

FILE(GLOB model_src *.h)    
 

INSTALL(FILES ${model_src} DESTINATION include/common/actuators/models/technosoftSimulator ) 

#INSTALL(FILES ${model_lib} DESTINATION lib/)

INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

