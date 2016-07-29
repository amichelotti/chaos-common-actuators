cmake_minimum_required(VERSION 2.6)
project(common_actuators)
FILE(GLOB core_src core/*.cpp)    
ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})

 INSTALL_TARGETS(/lib ${PROJECT_NAME})

FILE(GLOB core_src core/*.h)    
INSTALL(FILES ${core_src} DESTINATION include/common/actuators/core)
add_subdirectory(models)





