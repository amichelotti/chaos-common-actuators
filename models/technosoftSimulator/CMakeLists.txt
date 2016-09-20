cmake_minimum_required(VERSION 2.8)
project(common_actuators_technosoftSimulator)
MESG(STATUS "libraries ${model_lib}")
ADD_DEFINITIONS(-DDEBUG -fpic)

FILE(GLOB src *.cpp)    
ADD_LIBRARY(${PROJECT_NAME} STATIC ${src} )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_debug boost_regex boost_log boost_system boost_thread pthread)
ADD_EXECUTABLE(test_technosoftSim_slit test/test_slit.cpp)
TARGET_LINK_LIBRARIES(test_technosoftSim_slit ${PROJECT_NAME} common_actuators common_debug ${chaoslib}  boost_regex boost_log boost_system boost_thread pthread rt)


FILE(GLOB model_src *.h)    
 

INSTALL(FILES ${model_src} DESTINATION include/common/actuators/models/technosoftSimulator ) 

#INSTALL(FILES ${model_lib} DESTINATION lib/)
INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL_TARGETS(/bin test_technosoftSim_slit)

