cmake_minimum_required(VERSION 2.6)
project(common_actuators)

IF (DEFINED ENV{CHAOS_BUNDLE} )
  include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)
ELSE()
  MESSAGE(STATUS "STANDALONE, environment CHAOS_BUNDLE defined")
  IF (NOT CMAKE_INSTALL_PREFIX) 
    MESSAGE(fatal "you must add -DCMAKE_INSTALL_PREFIX=<dir>")
  ELSE()
    MESSAGE(STATUS "install prefix ${CMAKE_INSTALL_PREFIX}")
  ENDIF()
  INCLUDE_DIRECTORIES(. ../.. ${CMAKE_INSTALL_PREFIX}/include)
  LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/build ${CMAKE_INSTALL_PREFIX}/lib)
ENDIF()

FILE(GLOB core_src core/*.cpp)    
ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})

 INSTALL_TARGETS(/lib ${PROJECT_NAME})

FILE(GLOB core_src core/*.h)    
INSTALL(FILES ${core_src} DESTINATION include/common/actuators/core)
add_subdirectory(models)





