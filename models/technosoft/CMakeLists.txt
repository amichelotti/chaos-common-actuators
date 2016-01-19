cmake_minimum_required(VERSION 2.6)
project(common_actuators_technosoft)

IF (DEFINED ENV{CHAOS_BUNDLE} )
  include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)
set (chaoslib chaos_uitoolkit chaos_common )
ELSE()
  MESSAGE(STATUS "STANDALONE, environment CHAOS_BUNDLE defined")
  IF (NOT CMAKE_INSTALL_PREFIX) 
    MESSAGE(fatal "you must add -DCMAKE_INSTALL_PREFIX=<dir>")
  ELSE()
    MESSAGE(STATUS "install prefix ${CMAKE_INSTALL_PREFIX}")
  ENDIF()
  INCLUDE_DIRECTORIES(. ../../../../)
  LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/build ${CMAKE_INSTALL_PREFIX}/lib)
ENDIF()


ADD_DEFINITIONS()

FILE(GLOB src *.cpp)    
IF(BUILD_FORCE_STATIC)
  ADD_LIBRARY(${PROJECT_NAME} STATIC ${src} )
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${src})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_debug TML_lib )
ADD_EXECUTABLE(test_technosoft test/main.cpp)
TARGET_LINK_LIBRARIES(test_technosoft  ${PROJECT_NAME} common_debug ${chaoslib})


FILE(GLOB model_src *.h)    
 

INSTALL(FILES ${model_src} DESTINATION include/common/actuators/models/technosoft ) 
INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL_TARGETS(/bin test_technosoft)

