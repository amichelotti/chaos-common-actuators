cmake_minimum_required(VERSION 2.6)
project(common_actuators_technosoft)

IF (DEFINED ENV{CHAOS_BUNDLE} )
  include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)
set (chaoslib ${FrameworkLib} )
ELSE()
set (chaoslib boost_regex boost_log boost_thread boost_system pthread)
  MESSAGE(STATUS "STANDALONE, environment CHAOS_BUNDLE defined")
  IF (NOT CMAKE_INSTALL_PREFIX) 
    MESSAGE(fatal "you must add -DCMAKE_INSTALL_PREFIX=<dir>")
  ELSE()
    MESSAGE(STATUS "install prefix ${CMAKE_INSTALL_PREFIX}")
  ENDIF()
  INCLUDE_DIRECTORIES(. ../../../../ ../../../)
  LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/build ${CMAKE_INSTALL_PREFIX}/lib)
ENDIF()

LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)
ADD_DEFINITIONS()

FILE(GLOB src *.cpp)    
IF(BUILD_FORCE_STATIC)
  ADD_LIBRARY(${PROJECT_NAME} STATIC ${src} )
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${src})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_debug TML_lib tmlcomm)
ADD_EXECUTABLE(test_technosoft_slit test/test_slit.cpp)
TARGET_LINK_LIBRARIES(test_technosoft_slit ${PROJECT_NAME} common_actuators common_debug ${chaoslib} tmlcomm boost_regex )


FILE(GLOB model_src *.h)    
 

INSTALL(FILES ${model_src} DESTINATION include/common/actuators/models/technosoft ) 
FILE(GLOB model_lib lib/*)    
INSTALL(FILES ${model_lib} DESTINATION lib/)
INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL_TARGETS(/bin test_technosoft_slit)

